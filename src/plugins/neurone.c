/*
    Copyright (C) 2010-2014  EPFL (Ecole Polytechnique Fédérale de Lausanne)
    Laboratory CNBI (Chair in Non-Invasive Brain-Machine Interface)
    Nicolas Bourdaud <nicolas.bourdaud@epfl.ch>
    Serafeim Perdikis <serafeim.perdikis@epfl.ch>

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
#include <errno.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netdb.h>
#include <time.h>
#include <arpa/inet.h>
#include <regex.h>
#include <endian.h>
#include <stdio.h>      /* printf */
#include <eegdev-pluginapi.h>
#include "device-helper.h"

#pragma pack(1)
#ifndef ULONG
typedef uint32_t ULONG;
#endif


struct neurone_eegdev {
	struct devmodule dev;
	int datafd;
	int fs;
	unsigned int nch;
	struct egdi_chinfo* chmap;
	pthread_t thid;
	pthread_mutex_t mtx;
	bool cap_set;
	int port;
};

#define get_neurone(dev_p) ((struct neurone_eegdev*)(dev_p))



/******************************************************************
 *                Elements missing from old API                	  *
 ******************************************************************/


struct egdi_signal_info {
	const char *unit, *transducer, *prefiltering;
	int isint, bsc, dtype, mmtype;
	double scale;
	union gval min, max;
};


struct egdi_chinfo {
	const char *label;
	const struct egdi_signal_info* si;
	int stype;
};

/* EGDCAP_NOCP_*: use pointer directly (do not copy data) */
#define EGDCAP_NOCP_CHMAP    0x00000001
#define EGDCAP_NOCP_DEVID    0x00000002
#define EGDCAP_NOCP_DEVTYPE    0x00000004
#define EGDCAP_NOCP_CHLABEL    0x00000008

struct blockmapping {
	int nch;
	int num_skipped;
	int skipped_stype;
	const struct egdi_chinfo* chmap;
	const struct egdi_signal_info* default_info;
};

struct plugincap {
	unsigned int sampling_freq;
	int flags;
	unsigned int num_mappings;
	const struct blockmapping* mappings;
	const char* device_type;
	const char* device_id;
};

//struct egdi_optname {
//    const char *name, *defvalue;
//};


union device_buffer {
	int i;
	unsigned char c[4];
};


struct data_neurone_char {
	char FrameType;
	char MainUnitNum;
	char Reserved[2];
	char PacketSeqNo[4];
	char NumChannels[2];
	char NumSampleBundles[2];
	char FirstSampleIndex[8];
	char FirstSampleTime[8];
	char Samples[3000];
};


#define NUM_CHANNELS_TOTAL  81
#define NUM_CHANNELS_EEG  64
#define NUM_CHANNELS_EMG  16
#define NUM_CHANNELS_TRIG  1

int SAMPLING_RATE;
int BLOCK_SIZE;


static inline
void egdi_set_gval(union gval* dst, int type, double val)
{
	if (type == EGD_INT32)
		dst->valint32_t = val;
	else if (type == EGD_FLOAT)
		dst->valfloat = val;
	else if (type == EGD_DOUBLE)
		dst->valdouble = val;
}


/******************************************************************
 *                       neurone internals                     	  *
 ******************************************************************/

static const char analog_unit[] = "uV";
static const char trigger_unit[] = "Boolean";
static const char analog_transducter[] = "Active Electrode";
static const char trigger_transducter[] = "Triggers and Status";
static const char trigger_prefiltering[] = "No filtering";
static const char neurone_device_type[] = "BrainAmp Recorder/RecView";
static const char trilabel[8] = "triggers";

static const char channel_names_old[NUM_CHANNELS_TOTAL][8] = {
	"FP1", "FP2", "F3", "F4", "C3", "C4", "P3", "P4",
	"O1", "O2", "F7", "F8", "T7", "T8", "P7", "P8",
	"Fz", "Cz", "Pz", "Iz", "FC1", "FC2", "CP1", "CP2",
	"FC5", "FC6", "CP5", "CP6", "TP9", "TP10", "F9", "F10",
	"EXG1", "EXG2", "EXG3", "EXG4", "EXG5", "EXG6", "EXG7", "EXG8",
	"F1", "F2", "C1", "C2", "P1", "P2", "AF3", "AF4",
	"FC3", "FC4", "CP3", "CP4", "PO3", "PO4", "F5", "F6",
	"C5", "C6", "P5", "P6", "AF7", "AF8", "FT7", "FT8",
	"TP7", "TP8", "PO7", "PO8", "FPz", "CPz", "POz", "Oz",
	"EXG9", "EXG10", "EXG11", "EXG12", "EXG13", "EXG14", "EXG15", "EXG16"
};

// The system uses channels 1..32 for EEG, 33..40 for EXG, 41..72 for EEG, 73..80 for EXG. 
// We reorganize them so the first 64 will be EEG, although with the original numbering for better localization
static const char channel_names[NUM_CHANNELS_TOTAL][8] = {
	"1.FP1", "2.FP2", "3.F3", "4.F4", "5.C3", "6.C4", "7.P3", "8.P4",
	"9.O1", "10.O2", "11.F7", "12.F8", "13.T7", "14.T8", "15.P7", "16.P8",
	"17.Fz", "18.Cz", "19.Pz", "20.Iz", "21.FC1", "22.FC2", "23.CP1", "24.CP2",
	"25.FC5", "26.FC6", "27.CP5", "28.CP6", "29.TP9", "30.TP10", "31.F9", "32.F10",
	"41.F1", "42.F2", "43.C1", "44.C2", "45.P1", "46.P2", "47.AF3", "48.AF4",
	"49.FC3", "50.FC4", "51.CP3", "52.CP4", "53.PO3", "54.PO4", "55.F5", "56.F6",
	"57.C5", "58.C6", "59.P5", "60.P6", "61.AF7", "62.AF8", "63.FT7", "64.FT8",
	"65.TP7", "66.TP8", "67.PO7", "68.PO8", "69.FPz", "70.CPz", "71.POz", "72.Oz",
	"33.EXG1", "34.EXG2", "35.EXG3", "36.EXG4", "37.EXG5", "38.EXG6", "39.EXG7", "40.EXG8",	
	"73.EXG9", "74.EXG10", "75.EXG11", "76.EXG12", "77.EXG13", "78.EXG14", "79.EXG15", "80.EXG16"
};

// We need to remap the indices
static const int channel_idx[NUM_CHANNELS_TOTAL] = {
	0, 1, 2, 3, 4, 5, 6, 7, 
	8,9, 10, 11, 12, 13, 14, 15, 
	16, 17, 18, 19, 20, 21, 22, 23, 
	24, 25, 26, 27, 28, 29, 30, 31, 
	64, 65, 66, 67, 68, 69, 70, 71,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 59, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59, 60, 61, 62, 63,
	72, 73, 74, 75, 76, 77, 78, 79,
	80
};


static
const struct egdi_signal_info neurone_siginfo[2] = {
	{
		.unit = "uV", .transducer = "Active electrode Float",
		.isint = 0, .bsc = 0, .dtype = EGD_FLOAT,
		.mmtype = EGD_FLOAT, .min = {.valfloat = -16384.0},
		.max = {.valfloat = 16384.0}
	}, {
		.unit = "uV", .transducer = "Triggers and Status",
		.isint = 1, .bsc = 0, .dtype = EGD_INT32,
		.mmtype = EGD_INT32, .min = {.valint32_t = -8388608},
		.max = {.valint32_t = 8388608}
	}
};

// static
// const struct egdi_signal_info neurone_siginfo[2] = {
// 	{
// 		.unit = "uV", .transducer = "Active electrode Float",
// 		.isint = 0, .bsc = 0, .dtype = EGD_DOUBLE,
// 		.mmtype = EGD_DOUBLE, .min = {.valdouble = -262144.0},
// 		.max = {.valdouble = 262143.96875}
// 	}, {
// 		.unit = "uV", .transducer = "Triggers and Status",
// 		.isint = 1, .bsc = 0, .dtype = EGD_INT32,
// 		.mmtype = EGD_INT32, .min = {.valint32_t = -8388608},
// 		.max = {.valint32_t = 8388608}
// 	}
// };


enum {SR_PARAM, PORT, NUMOPT};
static const struct egdi_optname neurone_options[] = {
	[SR_PARAM] = {.name = "samplingrate", .defvalue = "1000"},
	[PORT] = {.name = "port", .defvalue = "50000"},
	[NUMOPT] = {.name = NULL},
};


static
int neurone_set_capability(struct neurone_eegdev* neuronedev, const char* guid, unsigned int sf)
{
	char* myguid = "neurone";
	struct blockmapping neurone_mapping = {.num_skipped = 0};
	neurone_mapping.nch = neuronedev->nch; // Account for trigger channel
	neurone_mapping.chmap = neuronedev->chmap;
	neurone_mapping.default_info = neurone_siginfo;

	struct plugincap cap = {
		.sampling_freq = sf,
		.num_mappings = 1,
		.mappings = &neurone_mapping,
		.device_type = neurone_device_type,
		.device_id = myguid,
		.flags = EGDCAP_NOCP_CHMAP | EGDCAP_NOCP_DEVID | EGDCAP_NOCP_DEVTYPE | EGDCAP_NOCP_CHLABEL |
		EGD_CAP_FS | EGD_CAP_TYPELIST
	};
	// And now downgrade to the stupid older interface
	struct devmodule* dev = &neuronedev->dev;

	struct systemcap capold = {.type_nch = {0}};

	for (unsigned int i = 0; i < neuronedev->nch; i++)
		capold.type_nch[neuronedev->chmap[i].stype]++;
	capold.sampling_freq = sf;
	capold.device_type = neurone_device_type;
	capold.device_id = myguid;

	dev->ci.set_cap(dev, &capold);

	dev->ci.set_input_samlen(dev, ((int)neurone_mapping.nch)*sizeof(int32_t));

	return 0;
}

static
void* neurone_read_fn(void *data)
{
	struct neurone_eegdev* tdev = data;
	const struct core_interface* restrict ci = &tdev->dev.ci;


	int length_data_double;
	float* dataDst;
	float* auxDst;

	union device_buffer db;
	struct data_neurone_char *data_pckg;
	char message[9000];

	int size;
	int b, ch;
	int idx_packg = 0;
	int first_iteration = 1;

	while (1) {

		if (first_iteration) {
			first_iteration = 0;
			// All channels except trigger
			unsigned int nChannels = (ULONG) NUM_CHANNELS_TOTAL-1;
			tdev->nch = nChannels;
			int SamplingFreq = SAMPLING_RATE;
			pthread_mutex_lock(&tdev->mtx);
			tdev->fs = (int)SamplingFreq;
			struct egdi_chinfo *newchmap;
			newchmap = malloc(((tdev->nch) + 1) * sizeof(struct egdi_chinfo)); //Add an extra ch for the trigger
			tdev->chmap = newchmap;
			struct egdi_signal_info* siginf = malloc((nChannels + 1) * sizeof(struct egdi_signal_info));

			// First package may be empty
			size = recv(tdev->datafd, message, sizeof(struct data_neurone_char), 0);
			if (size<=0) {
				size = recv(tdev->datafd, message, sizeof(struct data_neurone_char), 0);
			}
			if (size<=0) {
				fprintf(stderr, "Wrong packet!\n");
				exit(-1);
			}

			db.c[0] = message[11];
			db.c[1] = message[10];
			db.c[2] = 0;
			db.c[3] = 0;
			BLOCK_SIZE = db.i;
			printf("Estimated Block Size: %d\n", BLOCK_SIZE);

			length_data_double = (NUM_CHANNELS_TOTAL) * BLOCK_SIZE * sizeof(float);
			dataDst = malloc(length_data_double);

			for (ULONG i = 0; i < nChannels; i++)
			{
				// Retrieve channel names and resolutions.
				char* trigStr = channel_names[i];
				char* trigLbl = (char*)malloc((strlen(trigStr) + 1) * sizeof(char));
				memcpy(trigLbl, trigStr, strlen(trigStr) + 1);
				tdev->chmap[i].label = (char*)malloc((strlen(trigLbl) + 1) * sizeof(char));
				// Set channel labels
				tdev->chmap[i].label = trigLbl;
				if (i<NUM_CHANNELS_EEG) {
					tdev->chmap[i].stype = EGD_EEG;	
				} else {
					tdev->chmap[i].stype = EGD_SENSOR;					
				}

				siginf[i] = neurone_siginfo[0];
				siginf[i].scale = 1.0; //Scale individually for each channel
				tdev->chmap[i].si = &siginf[i];
			}

			// Now augment channels by one to accommodate trigger channel
			tdev->nch = tdev->nch + 1;

			// Add info for trigger channel
			char* trigStr = "triggers";
			char* trigLbl = (char*)malloc((strlen(trigStr)+1)*sizeof(char));
			memcpy(trigLbl,trigStr,strlen(trigStr)+1);
			tdev->chmap[nChannels].label = trigLbl;

			tdev->chmap[nChannels].stype = EGD_TRIGGER;
			siginf[nChannels] = neurone_siginfo[0];
			siginf[nChannels].scale = 1.0;
			tdev->chmap[nChannels].si = &siginf[nChannels];

			// Set device capabilities
			neurone_set_capability(tdev, 0, (unsigned int)SamplingFreq);
			tdev->cap_set = true;
			pthread_mutex_unlock(&tdev->mtx);
		}


		auxDst = dataDst;
		size = recv(tdev->datafd, message, sizeof(struct data_neurone_char), 0);
		// printf("Size is %d\n", size);
		if (size > 0) {
			idx_packg=0;
			data_pckg = (struct data_neurone_char *) message;
			for (b=0; b < BLOCK_SIZE; b++) {
				for (ch = 0; ch < NUM_CHANNELS_TOTAL; ch++) {
					db.c[0] = data_pckg->Samples[2 + idx_packg * 3];
					db.c[1] = data_pckg->Samples[1 + idx_packg * 3];
					db.c[2] = data_pckg->Samples[0 + idx_packg * 3];
					db.c[3] = 0;
					// This is the trigger channel
					if ((ch+1)==NUM_CHANNELS_TOTAL) {
						auxDst[b*NUM_CHANNELS_TOTAL+channel_idx[ch]] = (int) db.c[1] / 1.0;
					} else {
						auxDst[b*NUM_CHANNELS_TOTAL+channel_idx[ch]] = ((float) db.i) / 10.0;
					}
					idx_packg++;
				}
			}
			ci->update_ringbuffer(&(tdev->dev), dataDst, length_data_double);
		}

		
	}
	// We can reach here only if there was an error previously
	ci->report_error(&tdev->dev, errno);
	return NULL;
}



/*****************************************************************
 *                        neurone misc                            *
 *****************************************************************/


static
int connect_server(int port)
{

	int sockfd;
	char buffer[256];
	struct sockaddr_in serv_addr;

	/* First call to socket() function */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	if (sockfd < 0) {
		fprintf(stderr, "ERROR opening socket");
		exit(1);
	}

	/* Initialize socket structure */
	bzero((char *) &serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);

	/* Now bind the host address using bind() call.*/
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		perror("ERROR on binding");
		exit(1);
	}

	// printf("Socket binded %d\n", sockfd);

	return sockfd;


}

static
int init_data_com(struct neurone_eegdev* tdev, int port)
{
	struct timespec tim;
	tim.tv_sec = 0;
	tim.tv_nsec = 500000000;
	struct devmodule* dev = &tdev->dev;

	tdev->datafd = connect_server(port);
	tdev->port = port;
	if (tdev->datafd < 0) {
		close(tdev->datafd);
		tdev->datafd = -1;
		return -1;
	}
	// Give it some time
	nanosleep(&tim, NULL);

	int threadretval = pthread_create(&tdev->thid, NULL, neurone_read_fn, tdev);
	if (threadretval < 0) {
		close(tdev->datafd);
		tdev->datafd = -1;
		return -1;
	}

	return 0;
}

/******************************************************************
 *               neurone methods implementation                	  *
 ******************************************************************/

static
int neurone_close_device(struct devmodule* dev)
{
	struct neurone_eegdev* tdev = get_neurone(dev);
	unsigned int i;

	// Free channels metadata
	free(tdev->chmap);

	// Destroy data connection
	if (tdev->datafd >= 0) {
		pthread_cancel(tdev->thid);
		pthread_join(tdev->thid, NULL);
		close(tdev->datafd);
	}
}

static
int neurone_open_device(struct devmodule* dev, const char* optv[])
{
	struct neurone_eegdev* tdev = get_neurone(dev);
	int port = atoi(optv[PORT]);
	SAMPLING_RATE = atoi(optv[SR_PARAM]);
	
	printf("Sampling frequency: %d Hz\n", SAMPLING_RATE);
	printf("Listening Address: 192.168.200.240:%d\n", port);


	tdev->datafd = -1;
	pthread_mutex_lock(&tdev->mtx);
	tdev->cap_set = false;
	pthread_mutex_unlock(&tdev->mtx);

	if ( init_data_com(tdev, port) )
	{
		neurone_close_device(dev);
		return -1;
	}

	// Wait here until we are sure all is set and running
	// This is due to the fact that I need to read the first
	// message in order to setup the device
	bool allset = false;
	while (true) {
		pthread_mutex_lock(&tdev->mtx);
		allset = tdev->cap_set;
		pthread_mutex_unlock(&tdev->mtx);
		if (allset) {
			break;
		}
	}

	return 0;
}

static
int neurone_set_channel_groups(struct devmodule* dev, unsigned int ngrp,
                            const struct grpconf* grp)
{
	struct neurone_eegdev* tdev = get_neurone(dev);
	pthread_mutex_lock(&tdev->mtx);
	struct selected_channels* selch;
	int i, nsel = 0;

	// Downgrade from egdi_chinfo to egdich struct
	struct egdich oldchmap[tdev->nch];
	for (unsigned int i = 0; i < tdev->nch; i++) {
		oldchmap[i].label = tdev->chmap[i].label;
		oldchmap[i].stype = tdev->chmap[i].stype;
		oldchmap[i].dtype = tdev->chmap[i].si->dtype;
		oldchmap[i].data = (void*)tdev->chmap[i].si;

	}
	nsel = egdi_split_alloc_chgroups(dev, oldchmap,
	                                 ngrp, grp, &selch);
	for (i = 0; i < nsel; i++)
		selch[i].bsc = 0;
	pthread_mutex_unlock(&tdev->mtx);
	return (nsel < 0) ? -1 : 0;
}


static
void neurone_fill_chinfo(const struct devmodule* dev, int stype,
                      unsigned int ich, struct egd_chinfo* info)
{
	struct neurone_eegdev* tdev = get_neurone(dev);
	pthread_mutex_lock(&tdev->mtx);

	pthread_mutex_unlock(&tdev->mtx);
	if (stype != EGD_TRIGGER) {
		if (stype == EGD_EEG) {
		info->label = tdev->chmap[ich].label;
		} else {
			char sensorstr[10];
			char ichstr[3];
			strcpy(sensorstr, "EXG");
			sprintf(ichstr, "%d", (ich + 1));
			strcat(sensorstr, ichstr);
			info->label = sensorstr;
		}

		info->isint = 0;
		info->dtype = EGD_FLOAT;
		info->min.valfloat = -16384.0;
		info->max.valfloat = +16384.0;
		info->unit = analog_unit;
		info->transducter = analog_transducter;
		info->prefiltering = "Unknown";

		// info->isint = 0;
		// info->dtype = EGD_DOUBLE;
		// info->min.valdouble = -262144.0;
		// info->max.valdouble = 262143.96875;
		// info->unit = analog_unit;
		// info->transducter = analog_transducter;
		// info->prefiltering = "Unknown";


	} else {
		info->label = "triggers";
		info->isint = 1;
		info->dtype = EGD_INT32;
		info->min.valint32_t = -8388608;
		info->max.valint32_t = 8388607;
		info->unit = trigger_unit;
		info->transducter = trigger_transducter;
		info->prefiltering = trigger_prefiltering;
	}

}

API_EXPORTED
const struct egdi_plugin_info eegdev_plugin_info = {
	.plugin_abi = 	EEGDEV_PLUGIN_ABI_VERSION,
	.struct_size = 	sizeof(struct neurone_eegdev),
	.open_device = 		neurone_open_device,
	.close_device = 	neurone_close_device,
	.set_channel_groups = 	neurone_set_channel_groups,
	.fill_chinfo = 		neurone_fill_chinfo,
	.supported_opts = 	neurone_options
};

