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
//#include <sys/socket.h>
//#include <bluetooth/bluetooth.h>
//#include <bluetooth/rfcomm.h>

#include <eegdev-pluginapi.h>
#include <math.h>   

#include "ftd2xx.h"
#include <float.h>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

struct q20_eegdev {

	struct devmodule dev;
	FT_HANDLE ftHandle1;
	char * serialNumber;

	pthread_t thread_id;
	pthread_mutex_t acqlock;
	unsigned int runacq;

};

#define get_q20(dev_p) ((struct q20_eegdev*)(dev_p))

#define DEFAULT_SERIALNUMBER "AL1SFXV4"

/******************************************************************
 *                       q20 internals                     	  *
 ******************************************************************/
#define 	Xon		((BYTE)0x12) // No impedance check, if wanted change to 0x11
#define 	Xoff 	((BYTE)0x13)
#define 	NCH 	19

static const char q20label[NCH][10]= {
	"F7", "Fp1", "Fp2", "F8", "F3", "Fz", "F4",
	"C3", "Cz", "P8", "P7", "Pz", "P4", "T3", 
	"P3", "O1", "O2", "C4", "T4"
};

//static const char * q20labels = "Fp1,Fp2,Fz,F3,F4,F7,F8,Cz,C3,C4,T3,T4,T5,T6,Pz,P3,P4,O1,O2,A1,A2";

static const char q20unit[] = "uV";
static const char q20transducter[] = "Dry electrode";
	
static const union gval q20_scales[EGD_NUM_DTYPE] = {
	[EGD_INT32] = {.valint32_t = 1},
	[EGD_FLOAT] = {.valfloat = 0.00038805f},
	[EGD_DOUBLE] = {.valdouble = 0.00038805}
};
static const int q20_provided_stypes[] = {EGD_EEG};

enum {OPT_SERIALNUMBER, NUMOPT};

static const struct egdi_optname q20_options[] = {
	[OPT_SERIALNUMBER] = {.name = "serialNumber", .defvalue = DEFAULT_SERIALNUMBER},
	[NUMOPT] = {.name = NULL}
};


#define BUF_SIZE 33
char *int2bin(int a, char *buffer, int buf_size) {
    buffer += (buf_size - 1);

    for (int i = 31; i >= 0; i--) {
        *buffer-- = (a & 1) + '0';

        a >>= 1;
    }

    return buffer;
}


// Read a single BYTE
BYTE ReadBYTE(FT_HANDLE *ftHandle)
{
	DWORD BYTEsRead = 0;
	BYTE t_data;

	FT_Read(ftHandle, &t_data, 1, &BYTEsRead);

	return t_data;
}

static int q20_set_capability(struct q20_eegdev* q20dev, const char* serialNumber)
{
	struct systemcap cap = {
		.sampling_freq = 500, 
		.type_nch = {[EGD_EEG] = NCH},
		.device_type = "quick-20",
		.device_id = serialNumber
	};

	struct devmodule* dev = &q20dev->dev;

	dev->ci.set_cap(dev, &cap);
	dev->ci.set_input_samlen(dev, NCH*sizeof(double));
	return 0;
}

static void* q20_read_fn(void* arg)
{
	
	struct q20_eegdev* q20dev = arg;
	const struct core_interface* restrict ci = &q20dev->dev.ci;

	int runacq;
	double databuffer[NCH];
	size_t samlen = sizeof(databuffer);

	uint32_t msb, lsb2, lsb1;
	int32_t tempACC, impStatus, batteryBYTE, trigger;

	while (1) {

		//wait for sync BYTE 0xFF
		while (ReadBYTE(q20dev->ftHandle1) != 255) {};

		pthread_mutex_lock(&(q20dev->acqlock));
		runacq = q20dev->runacq;
		pthread_mutex_unlock(&(q20dev->acqlock));
		if (!runacq)
			break;

		//read packet counter
		int32_t packetCount = (int32_t)ReadBYTE(q20dev->ftHandle1);

		//read the 20 EEG channels
		for (int c = 0; c < NCH; c++){
			msb =  (uint32_t)ReadBYTE(q20dev->ftHandle1);
			lsb2 = (uint32_t)ReadBYTE(q20dev->ftHandle1);
			lsb1 = (uint32_t)ReadBYTE(q20dev->ftHandle1);

			//msb =  ReadBYTE(q20dev->ftHandle1);
			//lsb2 = ReadBYTE(q20dev->ftHandle1);
			//lsb1 = ReadBYTE(q20dev->ftHandle1);

			databuffer[c] = ((double)((msb << 24) | (lsb2 << 17) | (lsb1 << 10)));
			
			/*
			if (c==0)
			{	
				printf("%s\n", "start");
				//printf("%d\n", msb);
				printf("%d\n", lsb1);
				printf("%d\n", lsb1bis);
				//uint32_t shifts = ((msb << 24) + (lsb2 << 17) + (lsb1 << 10));
				//char buffer[BUF_SIZE];
    			//buffer[BUF_SIZE - 1] = '\0';
    			//int2bin(shifts, buffer, BUF_SIZE - 1);
    			//printf("%d\n", (int32_t)shifts);
    			//printf("%f\n",(float)((int32_t)shifts) );
    			//printf("%f\n",(float)shifts*(5/3)*(1/(pow(2,32)))*pow(10,6));	
    			
				//printf("%d\n", lsb2);
				//printf("%d\n", lsb1);
				//printf("%f\n", databuffer[c]);
			} */
		}

		//read the 3 ACC channels
		int NumACC = 3;
		for (int c = 0; c < NumACC; c++){
			msb =  ReadBYTE(q20dev->ftHandle1);
			lsb2 = ReadBYTE(q20dev->ftHandle1);
			lsb1 = ReadBYTE(q20dev->ftHandle1);

			tempACC = (msb << 24) | (lsb2 << 17) | (lsb1 << 10);
		}

		//read packet tail
		impStatus = ReadBYTE(q20dev->ftHandle1);

		//read battery voltage
		batteryBYTE = ReadBYTE(q20dev->ftHandle1);

		//read trigger
		trigger = (ReadBYTE(q20dev->ftHandle1)<<8) + ReadBYTE(q20dev->ftHandle1);

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
	q20dev->serialNumber = (char*)optv[OPT_SERIALNUMBER];

	FT_STATUS ftStatus;

	// Create device
	ftStatus = FT_OpenEx(q20dev->serialNumber, FT_OPEN_BY_SERIAL_NUMBER, &q20dev->ftHandle1);
	

	if (ftStatus == FT_OK) {
		
		ftStatus = FT_SetFlowControl(q20dev->ftHandle1, FT_FLOW_RTS_CTS, Xon, Xoff);
		if(ftStatus != FT_OK){
			printf("%s\n", "FAILED FT_SetFlowControl");}
		
		ftStatus = FT_SetDataCharacteristics(q20dev->ftHandle1, FT_BITS_8, FT_STOP_BITS_1,FT_PARITY_NONE);
		if(ftStatus != FT_OK){
			printf("%s\n", "FAILED FT_SetDataCharacteristics");}
		
		ftStatus = FT_SetBaudRate(q20dev->ftHandle1, ((unsigned int)3000000));
		if(ftStatus != FT_OK){
			printf("%s\n", "FAILED FT_SetBaudRate");}

		ftStatus = FT_SetLatencyTimer(q20dev->ftHandle1, 2);
		if(ftStatus != FT_OK){
			printf("%s\n", "FAILED FT_SetLatencyTimer");}	

		q20_set_capability(q20dev, q20dev->serialNumber);
		pthread_mutex_init(&(q20dev->acqlock), NULL);
		q20dev->runacq = 1;}
	else {
		goto error;
	}

	int ret;

	if (ret = pthread_create(&(q20dev->thread_id), NULL, q20_read_fn, q20dev))
		goto error;

	return 0;

error:
	return -1;
}


static
int q20_close_device(struct devmodule* dev)
{
	struct q20_eegdev* q20dev = get_q20(dev);

	pthread_mutex_lock(&(q20dev->acqlock));
	q20dev->runacq = 0;
	pthread_mutex_unlock(&(q20dev->acqlock));

	pthread_join(q20dev->thread_id, NULL);
	pthread_mutex_destroy(&(q20dev->acqlock));

	FT_Close(q20dev->ftHandle1);


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
	info->min.valdouble = -DBL_MAX;
	info->max.valdouble = DBL_MAX;
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

