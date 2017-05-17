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

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <stdint.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include "DSI.h"
#include <eegdev-pluginapi.h>

struct wsdsi_eegdev {

	struct devmodule dev;
	DSI_Headset h;

	pthread_t thread_id;
	pthread_mutex_t acqlock;
	unsigned int runacq;

	unsigned int samplesPerBatch;
	double bufferAheadSec;
	const char * serialport;
};

#define get_wsdsi(dev_p) ((struct wsdsi_eegdev*)(dev_p))

#define DEFAULT_PORT	"/dev/rfcomm40"
#define DEFAULT_VERBOSITY	"2"
#define DEFAULT_SAMPLEBATCH "1"
#define DEFAULT_BUFFERAHEADSEC "0.01"

/******************************************************************
 *                       wsdsi internals                     	  *
 ******************************************************************/
#define CODE	0xB0
#define EXCODE 	0x55
#define SYNC 	0xAA
#define NCH 	21

static const char wsdsilabel[NCH][10]= {
	"Fp1", "Fp2", "Fz", "F3", "F4", "F7", "F8",
	"Cz", "C3", "C4", "T3", "T4", "T5", "T6",
	"Pz", "P3", "P4", "O1", "O2", "A1", "A2"
};

static const char * wsdsilabels = "Fp1,Fp2,Fz,F3,F4,F7,F8,Cz,C3,C4,T3,T4,T5,T6,Pz,P3,P4,O1,O2,A1,A2";

static const char wsdsiunit[] = "uV";
static const char wsdsitransducter[] = "Dry electrode";
	
static const union gval wsdsi_scales[EGD_NUM_DTYPE] = {
	[EGD_INT32] = {.valint32_t = 1},
	[EGD_FLOAT] = {.valfloat = 1.0f},	// in uV
	[EGD_DOUBLE] = {.valdouble = 1.0}	// in uV
};
static const int wsdsi_provided_stypes[] = {EGD_EEG};

enum {OPT_VERBOSITY, OPT_SERIALPORT, OPT_SAMPLEBATCH, OPT_BUFFERAHEADSEC, NUMOPT};
static const struct egdi_optname wsdsi_options[] = {
	[OPT_VERBOSITY] = {.name = "verbosity", .defvalue = DEFAULT_VERBOSITY},
	[OPT_SERIALPORT] = {.name = "serialport", .defvalue = DEFAULT_PORT},
	[OPT_SAMPLEBATCH] = {.name = "samplesPerBatch", .defvalue = DEFAULT_SAMPLEBATCH},
	[OPT_BUFFERAHEADSEC] = {.name = "bufferAheadSec", .defvalue = DEFAULT_BUFFERAHEADSEC},
	[NUMOPT] = {.name = NULL}
};


int Message( const char * msg, int debugLevel )
    {
        return fprintf( stderr, "DSI Message (level %d): %s\n", debugLevel, msg );
    }

int CheckError( void )
    {
        if( DSI_Error() ) return fprintf( stderr, "%s\n", DSI_ClearError() );
        else return 0;
    }

#define CHECK     if( CheckError() != 0 ) return -1;

static
int wsdsi_set_capability(struct wsdsi_eegdev* wsdsidev, const char* serialport)
{
	struct systemcap cap = {
		.sampling_freq = 300, 
		.type_nch = {[EGD_EEG] = NCH},
		.device_type = "wsdsi",
		.device_id = serialport
	};
	struct devmodule* dev = &wsdsidev->dev;

	dev->ci.set_cap(dev, &cap);
	dev->ci.set_input_samlen(dev, NCH*sizeof(double)*wsdsidev->samplesPerBatch);
	return 0;
}

static void* wsdsi_read_fn(void* arg)
{
	
	struct wsdsi_eegdev* wsdsidev = arg;
	const struct core_interface* restrict ci = &wsdsidev->dev.ci;

	int runacq, ns;
	double databuffer[NCH*wsdsidev->samplesPerBatch];
	size_t samlen = NCH*sizeof(double)*wsdsidev->samplesPerBatch;
	uint8_t c, pLength;

	int targetExcessSamples = ( int )( 0.5 + DSI_Headset_GetSamplingRate( wsdsidev->h ) * wsdsidev->bufferAheadSec );
	printf("%s\n", "SP SP SP SP");
	printf("%f\n", wsdsidev->bufferAheadSec);

	DSI_Headset_ConfigureBatch( wsdsidev->h, wsdsidev->samplesPerBatch, wsdsidev->bufferAheadSec );
	DSI_Headset_StartBackgroundAcquisition( wsdsidev->h );


	while (1) {
		pthread_mutex_lock(&(wsdsidev->acqlock));
		runacq = wsdsidev->runacq;
		pthread_mutex_unlock(&(wsdsidev->acqlock));
		if (!runacq)
			break;
		
		DSI_Headset_WaitForBatch( wsdsidev->h );

		for(int channelIndex = 0; channelIndex < DSI_Headset_GetNumberOfChannels( wsdsidev->h ); channelIndex++ ){
			DSI_Channel c = DSI_Headset_GetChannelByIndex( wsdsidev->h, channelIndex );
				for( int sampleIndex = 0; sampleIndex < wsdsidev->samplesPerBatch; sampleIndex++ ){
					// The background thread is filling the buffers. This is where you empty them:
					databuffer[channelIndex  + sampleIndex * wsdsidev->samplesPerBatch] = DSI_Channel_ReadBuffered( c );;
					//printf("%s\n","BROUUU BIS");
				}
			}
		/*
		printf( "%9.4f,%3lu,%3lu,%3d,%3d\n",
                DSI_Headset_SecondsSinceConnection( wsdsidev->h ),
                DSI_Headset_GetNumberOfBufferedSamples( wsdsidev->h ),
                DSI_Headset_GetNumberOfOverflowedSamples( wsdsidev->h ),
                wsdsidev->samplesPerBatch,
                targetExcessSamples
            );
*/
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
	wsdsidev->serialport = optv[OPT_SERIALPORT];
	wsdsidev->samplesPerBatch = atoi(optv[OPT_SAMPLEBATCH]);
	wsdsidev->bufferAheadSec = atof(optv[OPT_BUFFERAHEADSEC]);

	int ret;

	const char * dllname = "libDSI-Linux-x86_64.so";
	int load_error = Load_DSI_API( dllname);
	if( load_error < 0 ) return fprintf( stderr, "failed to load dynamic library \"%s\"\n", DSI_DYLIB_NAME( dllname ) );
    if( load_error > 0 ) return fprintf( stderr, "failed to import %d functions from dynamic library \"%s\"\n", load_error, DSI_DYLIB_NAME( dllname ) );

	// Create device
	wsdsidev->h = DSI_Headset_New(NULL); CHECK
	DSI_Headset_SetMessageCallback( wsdsidev->h, Message ); CHECK
	DSI_Headset_SetVerbosity( wsdsidev->h, atoi(DEFAULT_VERBOSITY) ); CHECK
	DSI_Headset_Connect( wsdsidev->h, wsdsidev->serialport ); CHECK
	DSI_Headset_ChooseChannels( wsdsidev->h, wsdsilabels, NULL, 0 ); CHECK
	fprintf( stderr, "%s\n", DSI_Headset_GetInfoString( wsdsidev->h ) ); CHECK
	wsdsi_set_capability(wsdsidev, wsdsidev->serialport);

	pthread_mutex_init(&(wsdsidev->acqlock), NULL);
	wsdsidev->runacq = 1;

	if ((ret = pthread_create(&(wsdsidev->thread_id), NULL, wsdsi_read_fn, wsdsidev)))
		goto error;

	return 0;

error:
	return -1;
}


static
int wsdsi_close_device(struct devmodule* dev)
{
	struct wsdsi_eegdev* wsdsidev = get_wsdsi(dev);
	DSI_Headset_SetSampleCallback( wsdsidev->h, NULL, NULL );
	DSI_Headset_StopDataAcquisition( wsdsidev->h );
	DSI_Headset_Idle( wsdsidev->h, 1.0 );
	DSI_Headset_Delete( wsdsidev->h );

	wsdsidev->runacq = 0;

	return 0;
}


static
int wsdsi_set_channel_groups(struct devmodule* dev, unsigned int ngrp,
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

	info->isint = 0;
	info->dtype = EGD_DOUBLE;
	info->min.valdouble = -512.0 * wsdsi_scales[EGD_DOUBLE].valdouble;
	info->max.valdouble = 511.0 * wsdsi_scales[EGD_DOUBLE].valdouble;
	info->label = wsdsilabel[ich];
	info->unit = wsdsiunit;
	info->transducter = wsdsitransducter;
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

