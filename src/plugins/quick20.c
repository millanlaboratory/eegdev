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
#include <ftd2xx.h>
#include <eegdev-pluginapi.h>

struct q20_eegdev {

	struct devmodule dev;
	FT_HANDLE ftHandle1;
	const char * serialNumber;

	pthread_t thread_id;
	pthread_mutex_t acqlock;
	unsigned int runacq;

	unsigned int samplesPerBatch;
	double bufferAheadSec;
};

typedef unsigned long DWORD, *PDWORD, *LPDWORD;
typedef unsigned char byte;

#define get_q20(dev_p) ((struct q20_eegdev*)(dev_p))

//#define DEFAULT_PORT	"/dev/rfcomm40"
//#define DEFAULT_VERBOSITY	"2"
#define DEFAULT_SAMPLEBATCH "1"
#define DEFAULT_BUFFERAHEADSEC "0.06"

/******************************************************************
 *                       q20 internals                     	  *
 ******************************************************************/
#define 	Xon		0x11
#define 	Xoff 	0x13
#define 	NCH 	20


static const char q20label[NCH][10]= {
	"Fp1", "Fp2", "Fz", "F3", "F4", "F7", "F8",
	"Cz", "C3", "C4", "T3", "T4", "T5", "T6",
	"Pz", "P3", "P4", "O1", "O2", "A1"
};

//static const char * q20labels = "Fp1,Fp2,Fz,F3,F4,F7,F8,Cz,C3,C4,T3,T4,T5,T6,Pz,P3,P4,O1,O2,A1,A2";

static const char q20unit[] = "uV";
static const char q20transducter[] = "Dry electrode";
	
static const union gval q20_scales[EGD_NUM_DTYPE] = {
	[EGD_INT32] = {.valint32_t = 1},
	[EGD_FLOAT] = {.valfloat = 1.0f},	// in uV
	[EGD_DOUBLE] = {.valdouble = 1.0}	// in uV
};
static const int q20_provided_stypes[] = {EGD_EEG};

enum {OPT_SERIALNUMBER, OPT_SAMPLEBATCH, OPT_BUFFERAHEADSEC, NUMOPT};
static const struct egdi_optname q20_options[] = {
	[OPT_SERIALNUMBER] = {.name = "serialNumber", .defvalue = DEFAULT_SERIALNUMBER},
	[OPT_SAMPLEBATCH] = {.name = "samplesPerBatch", .defvalue = DEFAULT_SAMPLEBATCH},
	[OPT_BUFFERAHEADSEC] = {.name = "bufferAheadSec", .defvalue = DEFAULT_BUFFERAHEADSEC},
	[NUMOPT] = {.name = NULL}
};

/*
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
*/


// Read a single byte
byte ReadByte(FT_HANDLE *ftHandle)
{
	DWORD bytesRead = 0;
	byte t_data;

	FT_Read(ftHandle, &t_data, 1, &bytesRead);

	return t_data;
}

static
int q20_set_capability(struct q20_eegdev* q20dev, const char* serialNumber)
{
	struct systemcap cap = {
		.sampling_freq = 500, 
		.type_nch = {[EGD_EEG] = NCH},
		.device_type = "quick-20",
		.device_id = serialNumber
	};
	struct devmodule* dev = &q20dev->dev;

	dev->ci.set_cap(dev, &cap);
	dev->ci.set_input_samlen(dev, NCH*sizeof(double)*q20dev->samplesPerBatch);
	return 0;
}

static void* q20_read_fn(void* arg)
{
	
	struct q20_eegdev* q20dev = arg;
	const struct core_interface* restrict ci = &q20dev->dev.ci;

	int runacq;
	int databuffer[NCH*q20dev->samplesPerBatch];
	size_t samlen = NCH*sizeof(double)*q20dev->samplesPerBatch;
	//byte c, pLength;
	
	while (1) {

		//wait for sync byte 0xFF
		while (ReadByte(q20dev->ftHandle1) != 255) {};
		//read packet counter
		int packetCount = ReadByte(q20dev->ftHandle1);

		pthread_mutex_lock(&(q20dev->acqlock));
		runacq = q20dev->runacq;
		pthread_mutex_unlock(&(q20dev->acqlock));

		if (!runacq)
			break;

		//read the 20 EEG channels
		int packetCount = ReadByte(q20dev->ftHandle1);
		for (int c = 0; c < NumEEG; c++){
			msb =  ReadByte(q20dev->ftHandle1);
			lsb2 = ReadByte(q20dev->ftHandle1);
			lsb1 = ReadByte(q20dev->ftHandle1);

			databuffer[c] = (msb << 24) | (lsb2 << 17) | (lsb1 << 10);
		}

		//read the 3 ACC channels
		int NumACC = 3;
		for (int c = 0; c < NumACC; c++){
			msb =  ReadByte(q20dev->ftHandle1);
			lsb2 = ReadByte(q20dev->ftHandle1);
			lsb1 = ReadByte(q20dev->ftHandle1);

			int tempACC = (msb << 24) | (lsb2 << 17) | (lsb1 << 10);
		}

		//read packet tail
		int impStatus = ReadByte(q20dev->ftHandle1);

		//read battery voltage
		int batteryByte = ReadByte(q20dev->ftHandle1);

		//read trigger
		int trigger = (ReadByte(q20dev->ftHandle1)<<8) + ReadByte(q20dev->ftHandle1);

		// Update the eegdev structure with the new data
		if (ci->update_ringbuffer(&(q20dev->dev), databuffer, samlen))
			break;
	}
	return NULL;
error:
	ci->report_error(&(q20dev->dev), EIO);
	return NULL;
}

/******************************************************************
 *               WQ20 methods implementation                	  *
 ******************************************************************/
static int q20_open_device(struct devmodule* dev, const char* optv[]){

	struct q20_eegdev* q20dev = get_q20(dev);
	q20dev->serialNumber = optv[OPT_SERIALNUMBER];
	q20dev->samplesPerBatch = atoi(optv[OPT_SAMPLEBATCH]);
	q20dev->bufferAheadSec = atof(optv[OPT_BUFFERAHEADSEC]);

	FT_STATUS ftStatus;

	// Create device
	ftStatus = FT_OpenEx(q20dev->serialNumber, FT_OPEN_BY_SERIAL_NUMBER, q20dev->ftHandle1)	

	if (ftStatus == FT_OK) {
		FT_SetFlowControl(q20dev->ftHandle, FT_FLOW_RTS_CTS, Xon, Xoff);
		FT_SetDataCharacteristics(q20dev->ftHandle, FT_BITS_8, FT_STOP_BITS_1,FT_PARITY_NONE);
		FT_SetBaudRate(q20dev->ftHandle, 3000000);
		FT_SetLatencyTimer(q20dev->ftHandle, 2);	

		q20_set_capability(q20dev, q20dev->serialNumber);
		pthread_mutex_init(&(q20dev->acqlock), NULL);
		q20dev->runacq = 1;}
	else {
		goto error;
	}

	if ((int ret = pthread_create(&(q20dev->thread_id), NULL, q20_read_fn, q20dev)))
		goto error;

	return 0;

error:
	return -1;
}


static
int q20_close_device(struct devmodule* dev)
{
	struct q20_eegdev* q20dev = get_q20(dev);
	DSI_Headset_SetSampleCallback( q20dev->h, NULL, NULL );
	DSI_Headset_StopDataAcquisition( q20dev->h );
	DSI_Headset_Idle( q20dev->h, 1.0 );
	DSI_Headset_Delete( q20dev->h );

	q20dev->runacq = 0;

	return 0;
}


static
int q20_set_channel_groups(struct devmodule* dev, unsigned int ngrp,
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
		selch[i].sc = q20_scales[grp[i].datatype];
		selch[i].typeout = grp[i].datatype;
		selch[i].iarray = grp[i].iarray;
		selch[i].arr_offset = grp[i].arr_offset;
	}
		
	return 0;
}


static void q20_fill_chinfo(const struct devmodule* dev, int stype,
	                     unsigned int ich, struct egd_chinfo* info)
{
	(void)dev;
	(void)stype;

	info->isint = 0;
	info->dtype = EGD_DOUBLE;
	info->min.valdouble = -512.0 * q20_scales[EGD_DOUBLE].valdouble;
	info->max.valdouble = 511.0 * q20_scales[EGD_DOUBLE].valdouble;
	info->label = q20label[ich];
	info->unit = q20unit;
	info->transducter = q20transducter;
}


API_EXPORTED
const struct egdi_plugin_info eegdev_plugin_info = {
	.plugin_abi = 	EEGDEV_PLUGIN_ABI_VERSION,
	.struct_size = 	sizeof(struct q20_eegdev),
	.open_device = 		q20_open_device,
	.close_device = 	q20_close_device,
	.set_channel_groups = 	q20_set_channel_groups,
	.fill_chinfo = 		q20_fill_chinfo,
	.supported_opts =	q20_options
};

