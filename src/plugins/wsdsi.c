/*
    Copyright (C) 2010-2012  EPFL (Ecole Polytechnique Fédérale de Lausanne)
    Laboratory CNBI (Chair in Non-Invasive Brain-Machine Interface)
    Nicolas Bourdaud <nicolas.bourdaud@epfl.ch>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#if HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <errno.h>

#include "DSI.h"
#include <eegdev-pluginapi.h>
#include <float.h>

typedef const char label4_t[4];

struct wsdsi_eegdev {

	struct devmodule dev;
	DSI_Headset h;

	pthread_t thread_id;
	pthread_mutex_t acqlock;
	unsigned int runacq;

	unsigned int samplesPerBatch;
	double bufferAheadSec;
	const char * port;
};

#define get_wsdsi(dev_p) ((struct wsdsi_eegdev*)(dev_p))

#define DEFAULT_PORT	"/dev/ttyUSB0"
#define DEFAULT_REF	"A1/2+A2/2"
#define DEFAULT_VERBOSITY	"2"
#define DEFAULT_SAMPLEBATCH "15"
#define DEFAULT_BUFFERAHEADSEC "0.06"

/******************************************************************
 *                       wsdsi internals                     	  *
 ******************************************************************/
#define CODE	0xB0
#define EXCODE 	0x55
#define SYNC 	0xAA
#define NCH 	19


static const char * wsdsilabels = "Fp1 Fp2 Fz F3 F4 F7 F8 Cz C3 C4 T3 T4 T5 T6 Pz P3 P4 O1 O2";

static label4_t wsdsilabel[NCH]= {
	"Fp1", "Fp2", "Fz", "F3", "F4", "F7", "F8",
	"Cz", "C3", "C4", "T3", "T4", "T5", "T6",
	"Pz", "P3", "P4", "O1", "O2"
};


static const char* wsdsiunit[] = {"uV", "Boolean"};
static const char* wsdsitransducter[] = {"Dry active electrode", "Trigger"};
	
static const union gval wsdsi_scales[EGD_NUM_DTYPE] = {
	[EGD_INT32] = {.valint32_t = 1},
	[EGD_FLOAT] = {.valfloat = 1.0f},	// in uV
	[EGD_DOUBLE] = {.valdouble = 1.0}	// in uV
};

enum {
  OPT_PORT,
  OPT_REF,
  OPT_VERBOSITY, 
  OPT_SAMPLEBATCH,
  OPT_BUFFERAHEADSEC,
  NUMOPT};

static const struct egdi_optname wsdsi_options[] = {
	[OPT_PORT] = {.name = "PORT", .defvalue = DEFAULT_PORT},
	[OPT_REF] = {.name = "REF", .defvalue = DEFAULT_REF},
	[OPT_VERBOSITY] = {.name = "VERBOSE", .defvalue = DEFAULT_VERBOSITY},
	[OPT_SAMPLEBATCH] = {.name = "SAMPLEBATCH", .defvalue = DEFAULT_SAMPLEBATCH},
	[OPT_BUFFERAHEADSEC] = {.name = "BUFFERAHEADSEC", .defvalue = DEFAULT_BUFFERAHEADSEC},
	[NUMOPT] = {.name = NULL}
};


int Message( const char * msg, int debugLevel )
    {
        return fprintf( stderr, "DSI Message (level %d): %s\n", debugLevel, msg );
    }

int Error( void )
    {
        if( DSI_Error() ) return fprintf( stderr, "%s\n", DSI_ClearError() );
        else return 0;
    }

#define CHECK	Error();

static int wsdsi_set_capability(struct wsdsi_eegdev* wsdsidev)
{
	struct systemcap cap = {
		.sampling_freq = 300, 
		.type_nch = {[EGD_EEG] = NCH},
		.device_type = "wsdsi 24 (Wearable Sensings)",
		.device_id = wsdsidev->port
	};
	struct devmodule* dev = &wsdsidev->dev;

	dev->ci.set_cap(dev, &cap);
	dev->ci.set_input_samlen(dev, NCH*sizeof(double));
	return 0;
}

static void* wsdsi_read_fn(void* arg)
{
	
	struct wsdsi_eegdev* wsdsidev = arg;
	const struct core_interface* restrict ci = &wsdsidev->dev.ci;

	int runacq, ns;
	double databuffer[NCH*wsdsidev->samplesPerBatch];
	size_t samlen = sizeof(databuffer);
	uint8_t c, pLength;

	unsigned int targetExcessSamples = ( int )( 0.5 + DSI_Headset_GetSamplingRate( wsdsidev->h ) * wsdsidev->bufferAheadSec );

	DSI_Headset_ConfigureBatch( wsdsidev->h, wsdsidev->samplesPerBatch, wsdsidev->bufferAheadSec ); CHECK
	DSI_Headset_StartBackgroundAcquisition( wsdsidev->h ); CHECK


	while (1) {
		runacq = wsdsidev->runacq;
		if (!runacq)
			break;
		
		DSI_Headset_WaitForBatch( wsdsidev->h ); CHECK

		for(int channelIndex = 0; channelIndex < DSI_Headset_GetNumberOfChannels( wsdsidev->h ); channelIndex++ ){
			DSI_Channel c = DSI_Headset_GetChannelByIndex( wsdsidev->h, channelIndex ); CHECK
				for( int sampleIndex = 0; sampleIndex < wsdsidev->samplesPerBatch; sampleIndex++ ){
					// The background thread is filling the buffers. This is where you empty them:
					databuffer[channelIndex * wsdsidev->samplesPerBatch  + sampleIndex ] = DSI_Channel_ReadBuffered( c ); CHECK
				}
			}
		
		// Update the eegdev structure with the new data
		if (ci->update_ringbuffer(&(wsdsidev->dev), databuffer, samlen))
			break;
	}
	return NULL;
error:
	ci->report_error(&(wsdsidev->dev), EIO);
	return NULL;
}


/******************************************************************
 *               WQ20 methods implementation                	  *
 ******************************************************************/
static int wsdsi_open_device(struct devmodule* dev, const char* optv[]){

	struct wsdsi_eegdev* wsdsidev = get_wsdsi(dev);
	wsdsidev->port = optv[OPT_PORT];
	wsdsidev->samplesPerBatch = atoi(optv[OPT_SAMPLEBATCH]);
	wsdsidev->bufferAheadSec = atof(optv[OPT_BUFFERAHEADSEC]);

	int ret;

	const char * dllname = "libdsi.so";
	int load_error = Load_DSI_API( dllname );
	if( load_error < 0 ) return fprintf( stderr, "failed to load dynamic library \"%s\"\n", DSI_DYLIB_NAME( dllname ) );
    if( load_error > 0 ) return fprintf( stderr, "failed to import %d functions from dynamic library \"%s\"\n", load_error, DSI_DYLIB_NAME( dllname ) );
	
	// Create device
	wsdsidev->h = DSI_Headset_New(NULL);
	DSI_Headset_SetMessageCallback( wsdsidev->h, Message );
	DSI_Headset_SetVerbosity( wsdsidev->h, atoi(optv[OPT_VERBOSITY]) ); CHECK 
	DSI_Headset_Connect( wsdsidev->h, wsdsidev->port ); CHECK
	DSI_Headset_ChooseChannels( wsdsidev->h, wsdsilabels, optv[OPT_REF], 1 ); CHECK 
	fprintf( stderr, "%s\n", DSI_Headset_GetInfoString( wsdsidev->h ) );
	wsdsi_set_capability(wsdsidev);

	pthread_mutex_init(&(wsdsidev->acqlock), NULL);
	wsdsidev->runacq = 1;

	if ((ret = pthread_create(&(wsdsidev->thread_id), NULL, wsdsi_read_fn, wsdsidev)))
		goto error;

	return 0;

error:
	return -1;
}

static int wsdsi_close_device(struct devmodule* dev)
{
	struct wsdsi_eegdev* wsdsidev = get_wsdsi(dev);
	pthread_mutex_lock(&wsdsidev->acqlock);
    wsdsidev->runacq = 0;
    pthread_mutex_unlock(&wsdsidev->acqlock);
	
	DSI_Headset_SetSampleCallback( wsdsidev->h, NULL, NULL ); CHECK
	DSI_Headset_StopDataAcquisition( wsdsidev->h ); CHECK
	DSI_Headset_Idle( wsdsidev->h, 1.0 ); CHECK
    sleep(1);
    //printf("%s\n", "");
	DSI_Headset_Delete( wsdsidev->h ); CHECK

	pthread_join(wsdsidev->thread_id, NULL);
    pthread_mutex_destroy(&wsdsidev->acqlock);

	return 0;
}

static int wsdsi_set_channel_groups(struct devmodule* dev, unsigned int ngrp,
					const struct grpconf* grp)
{
	unsigned int i;
	struct selected_channels* selch;
	
	if (!(selch = dev->ci.alloc_input_groups(dev, ngrp)))
		return -1;

	for (i=0; i<ngrp; i++) {
		// Set parameters of (eeg -> ringbuffer)
		selch[i].in_offset = grp[i].index*sizeof(double);
		selch[i].inlen = grp[i].nch*sizeof(double);
		selch[i].bsc = 1;
		selch[i].typein = EGD_DOUBLE;
		selch[i].sc = wsdsi_scales[grp[i].datatype];
		selch[i].typeout = grp[i].datatype;
		selch[i].iarray = grp[i].iarray;
		selch[i].arr_offset = grp[i].arr_offset;
	}
		
	return 0;
}


static void wsdsi_fill_chinfo(const struct devmodule* dev, int stype,
	                     unsigned int ich, struct egd_chinfo* info)
{
	(void)dev;
	(void)stype;

	int t = 0;
	info->isint = 0;
	info->dtype = EGD_DOUBLE;
	info->min.valdouble = -DBL_MAX; // * wsdsi_scales[EGD_DOUBLE].valdouble;
	info->max.valdouble = DBL_MAX; // * wsdsi_scales[EGD_DOUBLE].valdouble;
	info->label = wsdsilabel[ich];
	info->unit = wsdsiunit[t];
	info->transducter = wsdsitransducter[t];
}


API_EXPORTED
const struct egdi_plugin_info eegdev_plugin_info = {
	.plugin_abi = 	EEGDEV_PLUGIN_ABI_VERSION,
	.struct_size = 	sizeof(struct wsdsi_eegdev),
	.open_device = 		wsdsi_open_device,
	.close_device = 	wsdsi_close_device,
	.set_channel_groups = 	wsdsi_set_channel_groups,
	.fill_chinfo = 		wsdsi_fill_chinfo,
	.supported_opts =	wsdsi_options
};

