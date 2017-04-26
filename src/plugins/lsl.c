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
#include <lsl_c.h>

#include "device-helper.h"
#include <eegdev-pluginapi.h>

struct lsl_eegdev {
	struct devmodule dev;
	pthread_t thread_id;
	pthread_mutex_t acqlock;
	unsigned int runacq; 
	lsl_streaminfo streaminfo;	/* the streaminfo returned by the resolve call */
	lsl_streaminfo fullstreaminfo;	/* the full streaminfo with desc meta-data */
	lsl_inlet inlet;		/* a stream inlet to get samples from */
	int NChannelAll;		/* Total number of channels in the stream (EEG, EMG, etc) */
	int ChunkSize;			/* Chunk size, i.e., number of samples in a chunk/frame */
	char** ChannelLabel;		/* Channel labels */
	char** ChannelUnit;		/* Channel units */
	struct egdich* chmap;		/* Channel map */
};

#define get_lsl(dev_p) ((struct lsl_eegdev*)(dev_p))

static const struct egdi_optname lsl_options[] = {
	{.name = "name", .defvalue = "gUSBamp"},
	{.name = NULL}
};


static void* lsl_read_fn(void* arg)
{
	struct lsl_eegdev* lsldev = arg;
	const struct core_interface* restrict ci = &lsldev->dev.ci;
	int runacq, errcode;
	int ChunkSizeInSamples = (lsldev->ChunkSize == 0)?10:lsldev->ChunkSize;
	int DataBufferSizeInFloats =  ChunkSizeInSamples * lsldev->NChannelAll;
	int TimestampBufferSizeInFloats =  ChunkSizeInSamples;
	float databuffer[DataBufferSizeInFloats];
	double timestampbuffer[TimestampBufferSizeInFloats];
	
	while (1) {
		pthread_mutex_lock(&(lsldev->acqlock));
		runacq = lsldev->runacq;
		pthread_mutex_unlock(&(lsldev->acqlock));
		if (!runacq)
			break;			
		unsigned long NValues = lsl_pull_chunk_f(lsldev->inlet, databuffer, timestampbuffer, DataBufferSizeInFloats, TimestampBufferSizeInFloats, LSL_FOREVER, &errcode);
		if(errcode != 0)
			goto error;	
		
		// Update the eegdev structure with the new data
		if (ci->update_ringbuffer(&(lsldev->dev), databuffer, DataBufferSizeInFloats*sizeof(float)))
			break;
	
	}
	
	return NULL;
error:
	ci->report_error(&(lsldev->dev), EIO);
	return NULL;
}


static
int lsl_set_capability(struct lsl_eegdev* lsldev)
{
	struct systemcap cap = {
		.sampling_freq = lsl_get_nominal_srate(lsldev->streaminfo), 
		.type_nch = {0},
		.device_type = lsl_get_type(lsldev->streaminfo),
		.device_id = lsl_get_source_id(lsldev->streaminfo)
	};

	lsl_xml_ptr chn = lsl_child(lsl_child(lsl_get_desc(lsldev->fullstreaminfo),"channels"),"channel");
    	for (int c=0; c<lsldev->NChannelAll; c++) {
		char* chantype = lsl_child_value_n(chn,"type");
		if (strcmp(chantype,"EEG")==0) {
			cap.type_nch[EGD_EEG]++;
		} else {
			cap.type_nch[EGD_SENSOR]++;
		}
		
		lsldev->ChannelLabel[c] = lsl_child_value_n(chn,"label");
		lsldev->ChannelUnit[c] = lsl_child_value_n(chn,"unit");
        	chn = lsl_next_sibling(chn);
    	}

	struct devmodule* dev = &lsldev->dev;
	dev->ci.set_cap(dev, &cap);
	dev->ci.set_input_samlen(dev, lsldev->NChannelAll * sizeof(int32_t));
	return 0;
}

/******************************************************************
 *               LSL methods implementation                	  *
 ******************************************************************/
static
int lsl_open_device(struct devmodule* dev, const char* optv[])
{
	int errcode, ret;
	struct lsl_eegdev* lsldev = get_lsl(dev);
	char* name = optv[0];
	fprintf(stdout,"AAAA\n");
	// Open the device
	int resolveRes = lsl_resolve_byprop(&(lsldev->streaminfo), 1000, "type","EEG", 1, LSL_FOREVER);
	fprintf(stdout,"resolveRes = %d\n",resolveRes);
	if ( resolveRes < 0){
		return -1;
	}
	fprintf(stdout,"BBBB\n");
	int sf = (int)lsl_get_nominal_srate(lsldev->streaminfo);
	fprintf(stdout,"CCCC\n");
	if(sf%16 == 0){
		lsldev->ChunkSize = (int)(sf/16.0);
	}else if(sf%10 == 0){
		lsldev->ChunkSize = (int)(sf/10.0);
	}else{
		lsldev->ChunkSize = 0;
	}
	lsldev->NChannelAll = lsl_get_channel_count(lsldev->streaminfo);
	lsldev->ChannelLabel = (char**)malloc(lsldev->NChannelAll*sizeof(char*));
	lsldev->ChannelUnit = (char**)malloc(lsldev->NChannelAll*sizeof(char*));
	for(int i=0;i<lsldev->NChannelAll;i++){
		lsldev->ChannelLabel[i] = (char*)malloc(10*sizeof(char));
		lsldev->ChannelUnit[i] = (char*)malloc(10*sizeof(char));	
	}

	/* make an inlet to read data from the stream (buffer max. 360 second of data, chunking as defined above, automatic recovery enabled) */
	lsldev->inlet = lsl_create_inlet(lsldev->streaminfo, 360, lsldev->ChunkSize, 1);
	lsldev->fullstreaminfo = lsl_get_fullinfo(lsldev->inlet,LSL_FOREVER,&errcode);
	if (errcode != 0)
		return errcode;

	lsldev->chmap = malloc((lsldev->NChannelAll)*sizeof(*lsldev->chmap));
	for (int i=0; i<lsldev->NChannelAll; i++) {
	
		lsldev->chmap[i].dtype = EGD_FLOAT;
		lsldev->chmap[i].stype = EGD_EEG;
	}	

	lsl_set_capability(lsldev);

	/* subscribe to the stream (automatically done by push, but a nice way of checking early on that we can connect successfully) */
	lsl_open_stream(lsldev->inlet,LSL_FOREVER,&errcode);
	
	pthread_mutex_init(&(lsldev->acqlock), NULL);
	lsldev->runacq = 1;
	if ((ret = pthread_create(&(lsldev->thread_id), NULL, 
	                           lsl_read_fn, lsldev)))
		goto error;
	
	return 0;

error:
	return -1;
}


static
int lsl_close_device(struct devmodule* dev)
{
	struct lsl_eegdev* lsldev = get_lsl(dev);


	pthread_mutex_lock(&(lsldev->acqlock));
	lsldev->runacq = 0;
	pthread_mutex_unlock(&(lsldev->acqlock));

	pthread_join(lsldev->thread_id, NULL);
	pthread_mutex_destroy(&(lsldev->acqlock));
	
	/* destroy objects and free memory */
	lsl_destroy_streaminfo(lsldev->streaminfo);
	lsl_destroy_streaminfo(lsldev->fullstreaminfo);
	lsl_destroy_inlet(lsldev->inlet);

	for(int i=0;i<lsldev->NChannelAll;i++){
		free(lsldev->ChannelLabel[i]);
	}
	free(lsldev->ChannelLabel);
	free(lsldev->chmap);
	
	return 0;
}


static
int lsl_set_channel_groups(struct devmodule* dev, unsigned int ngrp,
					const struct grpconf* grp)
{
	struct lsl_eegdev* lsldev = get_lsl(dev);
	struct selected_channels* selch;
	int i, nsel;

	nsel = egdi_split_alloc_chgroups(dev, lsldev->chmap,
	                                 ngrp, grp, &selch);
	for (i=0; i<nsel; i++)
		selch[i].bsc = 0;
		
	return (nsel >= 0) ? 0 : -1;
}


static void lsl_fill_chinfo(const struct devmodule* dev, int stype,
	                     unsigned int ich, struct egd_chinfo* info)
{
	struct lsl_eegdev* lsldev = get_lsl(dev);
	(void)dev;
	(void)stype;

	info->isint = 0;
	info->dtype = EGD_FLOAT;
	info->min.valfloat = -16384.0;
	info->max.valfloat = +16384.0;
	info->label = lsldev->ChannelLabel[ich];
	info->unit = lsldev->ChannelUnit[ich];
	info->transducter = lsl_get_source_id(lsldev->streaminfo);
}


API_EXPORTED
const struct egdi_plugin_info eegdev_plugin_info = {
	.plugin_abi = 	EEGDEV_PLUGIN_ABI_VERSION,
	.struct_size = 	sizeof(struct lsl_eegdev),
	.open_device = 		lsl_open_device,
	.close_device = 	lsl_close_device,
	.set_channel_groups = 	lsl_set_channel_groups,
	.fill_chinfo = 		lsl_fill_chinfo,
	.supported_opts =	lsl_options
};

