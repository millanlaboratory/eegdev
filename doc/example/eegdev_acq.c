#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <eegdev.h>

#define NEEG		64
#define NSENS		8

#define NS_CHUNK	8
#define NS_TOTAL	500

struct systemcap devcap;
struct grpconf grp[3] = {
	[0] = {
	       .sensortype = EGD_EEG, .index = 0, 
	       .iarray = 0, .datatype = EGD_FLOAT,
	       .arr_offset = 0, .nch = NEEG
	},
	[1] = {
	       .sensortype = EGD_SENSOR, .index = 0, 
	       .iarray = 0, .datatype = EGD_FLOAT,
	       .arr_offset = NEEG*sizeof(float), .nch = NSENS
	},
	[2] = {
	       .sensortype = EGD_TRIGGER, .index = 0, 
	       .iarray = 1, .datatype = EGD_INT32,
	       .arr_offset = 0, .nch = 1
	}
};
size_t arrstrides[2];
int32_t* triggers = NULL;
float* analogch = NULL;


/* Adjust the groups and buffers size according to the capabilities of the
system and allocate the data buffers.
Returns 0 in case of success, -1 otherwise */
int setup_groups_buffers(void)
{
	/* Adjust the groups configuration if the numbers of requested
	channels for EEG and sensor are too big */
	if (grp[0].nch > devcap.eeg_nmax)
		grp[0].nch = devcap.eeg_nmax;

	if (grp[1].nch > devcap.sensor_nmax)
		grp[1].nch = devcap.sensor_nmax;

	grp[1].arr_offset = devcap.eeg_nmax*sizeof(float);

	/* Setup the strides so that we get packed data into the buffers */
	arrstrides[0] = (grp[0].nch + grp[1].nch) * sizeof(float);
	arrstrides[1] = sizeof(int32_t);
	
	/* Allocate the buffers */
	analogch = malloc(arrstrides[0]*NS_CHUNK);
	triggers = malloc(arrstrides[1]*NS_CHUNK);
	if ((analogch == NULL) || (triggers == NULL))
		return -1;

	return 0;
}


int run_acquisition_loop(struct eegdev* dev)
{
	int i,j, num_ch;

	num_ch = grp[0].nch + grp[1].nch;

	printf("Starting acquisition...\n");
	egd_start(dev);

	i = 0;
	while (i < 500) {
		/* Fill the buffers with the next NS_CHUNK samples */
		if (egd_get_data(dev, NS_CHUNK, analogch, triggers))
			return -1;
		
		/* Display the trigger value and the 3rd channel in each
		samples */
		for (j=0; j<NS_CHUNK; j++)
			printf("sample %04i - tri: 0x%08x ch3: %f\n",
			       i+j, triggers[j], analogch[j*num_ch+2]);

		i += NS_CHUNK;
	}

	printf("Stopping acquisition...\n");
	egd_stop(dev);

	return 0;
}


int main(int argc, char* argv[])
{
	struct eegdev* dev;
	int retcode = 1, opt;

	/* Process command line options */
	while ((opt = getopt(argc, argv, "e:s:")) != -1) {
		if (opt == 'e')
			grp[0].nch = atoi(optarg);
		else if (opt == 's')
			grp[1].nch = atoi(optarg);
		else {
			fprintf(stderr, "Usage: %s [-e num_eeg_ch] [-s num_sensor_ch]\n",argv[0]);
			return 2;
		}
	}

	/* Connect to the system */
	dev = egd_open_biosemi();
	if (dev == NULL) {
		fprintf(stderr, "Cannot open the device: %s\n", 
		        strerror(errno));
		return 1;
	}

	/* Get and display the capabilities of the system */
	egd_get_cap(dev, &devcap);
	printf("Activetwo device:\n"
	       "\tsampling frequency : %u Hz\n"
	       "\tNumber of EEG channels : %u\n"
	       "\tNumber of sensor channels : %u\n"
	       "\tNumber of trigger channels : %u\n",
	       devcap.sampling_freq, 
	       devcap.eeg_nmax,
	       devcap.sensor_nmax,
	       devcap.trigger_nmax);

	printf("Press ENTER to continue");
	getchar();

	/* Setup the acquisition transfer */
	printf("Setting up the acquisition...\n");
	if (setup_groups_buffers())
		goto exit;
	if (egd_acq_setup(dev, 2, arrstrides, 3, grp))
		goto exit;
	
	/* Start, run and stop the acquisition */
	if (run_acquisition_loop(dev))
		goto exit;

	retcode = 0;
exit:
	if (retcode)
		fprintf(stderr, "Error caught: %s\n", strerror(errno));

	/* Free data buffers */
	free(analogch);
	free(triggers);

	/* Close the connection to the system */
	egd_close(dev);

	return retcode;
}
