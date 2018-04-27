#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <eegdev-pluginapi.h>
#include <float.h>

#include "wrapper.h"

#define EEGO_SDK_BIND_STATIC

typedef const char label4_t[4];
typedef const char label8_t[8];

struct eego_eegdev {
  struct devmodule dev;

  int amplifiers_nb;
  int streams_id;
  int stream_nb_channels;
  eemagine_sdk_amplifier_info amplifier_info;

  int NUM_EEG_CH;
  int NUM_EXG_CH;
  int NUM_TRI_CH;
  int NUM_SAMPLE_COUNTER_CH;
  int NCH;
  
  unsigned int offset[EGD_NUM_STYPE];

  pthread_t thread_id;
  pthread_mutex_t acqlock;
  unsigned int runacq;
};

#define get_eego(dev_p) ((struct eego_eegdev*)(dev_p))

#define CODE 0xB0
#define EXCODE 0x55
#define SYNC 0xAA

#define DEFAULT_NCH_EEG "64"
#define DEFAULT_NCH_EXG "1"
#define DEFAULT_NCH_TRIG "1"
#define DEFAULT_SAMPLING_FREQ "512"
#define DEFAULT_DEVICE_TYPE "Antneuro"
#define DEFAULT_DEVICE_ID "N/A"

static label4_t eegolabel32[] = {
    "FP1", "FPZ", "FP2", "F7",  "F3",  "FZ",  "F4",  "F8",  "FC5", "FC1", "FC2",
    "FC6", "M1",  "T7",  "C3",  "CZ",  "C4",  "T8",  "M2",  "CP5", "CP1", "CP2",
    "CP6", "P7",  "P3",  "Pz",  "P4",  "P8",  "POZ", "O1",  "OZ",  "O2"};

static label4_t eegolabel64[] = {
    "FP1", "FPZ", "FP2", "F7",  "F3",  "FZ",  "F4",  "F8",  "FC5", "FC1", "FC2",
    "FC6", "M1",  "T7",  "C3",  "CZ",  "C4",  "T8",  "M2",  "CP5", "CP1", "CP2",
    "CP6", "P7",  "P3",  "Pz",  "P4",  "P8",  "POZ", "O1",  "OZ",  "O2",  "AF7",
    "AF3", "AF4", "AF8", "F5",  "F1",  "F2",  "F6",  "FC3", "FCZ", "FC4", "C5",
    "C1",  "C2",  "C6",  "CP3", "CPZ", "CP4", "P5",  "P1",  "P2",  "P6",  "PO5",
    "PO3", "PO4", "PO6", "FT7", "FT8", "TP7", "TP8", "PO7", "PO8"};

static label8_t eegolabel128[] = {
    "FP1",   "FPZ",    "FP2",   "F7",     "F3",    "FZ",     "F4",     "F8",
    "FC5",   "FC1",    "FC2",   "FC6",    "M1",    "T7",     "C3",     "CZ",
    "C4",    "T8",     "M2",    "CP5",    "CP1",   "CP2",    "CP6",    "P7",
    "P3",    "Pz",     "P4",    "P8",     "POZ",   "O1",     "OZ",     "O2",
    "AF7",   "AF3",    "AF4",   "AF8",    "F5",    "F1",     "F2",     "F6",
    "FC3",   "FCZ",    "FC4",   "C5",     "C1",    "C2",     "C6",     "CP3",
    "CPZ",   "CP4",    "P5",    "P1",     "P2",    "P6",     "PO5",    "PO3",
    "PO4",   "PO6",    "FT7",   "FT8",    "TP7",   "TP8",    "PO7",    "PO8",
    "FT9",   "FT10",   "TPP9H", "TPP10H", "PO9",   "PO10",   "P9",     "P10",
    "AFF1",  "AFZ",    "AFF2",  "FFC5H",  "FFC3H", "FFC4H",  "FFC6H",  "FCC5H",
    "FCC3H", "FCC4H",  "FCC6H", "CCP5H",  "CPP3H", "CPP4H",  "CPP6YH", "PPO1",
    "PPO2",  "I1",     "IZ",    "I2",     "AFP3H", "AFP4H",  "AFF5H",  "AFF6H",
    "FFT7H", "FFC1H",  "FFC2H", "FFT8H",  "FTT9H", "FTT7H",  "FCC1H",  "FCC2H",
    "FTT8H", "FTT10H", "TTP7H", "CCP1H",  "CCP2H", "TTP8H",  "TPP7H",  "CPP1H",
    "CPP2H", "TPP8H",  "PPO9H", "PPO5H",  "PPO6H", "PPO10H", "POO9H",  "POO3H",
    "POO4H", "POO10H", "OI1H",  "OI2H"};

static const char sensorlabel[][8] = {
    "sens1",  "sens2",  "sens3",  "sens4",  "sens5",  "sens6",
    "sens7",  "sens8",  "sens9",  "sens10", "sens11", "sens12",
    "sens13", "sens14", "sens15", "sens16", "sens17", "sens18",
    "sens19", "sens20", "sens21", "sens22", "sens23", "sens24"};

static const char trigglabel[] = "Status";
static const char eego_device_type[] = "Antneuro";
static const char eego_device_id[] = "eego";
static const char* eegounit[] = {"uV", "Boolean"};
static const char* eegotransducter[] = {"Active electrode",
                                        "Trigger and Status"};


static const union gval eego_scales[EGD_NUM_DTYPE] = {
    [EGD_INT32] = {.valint32_t = 1},
    [EGD_FLOAT] = {.valfloat = 1000000.0f},  // in uV
    [EGD_DOUBLE] = {.valdouble = 1000000}    // in uV
};

enum {
  OPT_SR,
  OPT_NCH_EEG,
  OPT_NCH_SENSOR,
  OPT_NCH_TRIG,
  OPT_DEVICE_TYPE,
  OPT_DEVICE_ID,
  NUMOPT
};
static const struct egdi_optname eego_options[] = {
    [OPT_SR] = {.name = "SR", .defvalue = DEFAULT_SAMPLING_FREQ},
    [OPT_NCH_EEG] = {.name = "NCH_EEG", .defvalue = DEFAULT_NCH_EEG},
    [OPT_NCH_SENSOR] = {.name = "NCH_EXG", .defvalue = DEFAULT_NCH_EXG},
    [OPT_NCH_TRIG] = {.name = "NCH_TRIG", .defvalue = DEFAULT_NCH_TRIG},
    [OPT_DEVICE_TYPE] = {.name = "DEVICE_TYPE",
                         .defvalue = DEFAULT_DEVICE_TYPE},
    [OPT_DEVICE_ID] = {.name = "DEVICE_ID", .defvalue = DEFAULT_DEVICE_ID},
    [NUMOPT] = {.name = NULL}};


/**
 * @brief      Checks the output status of the SDK functions and dumps.
 *
 * @param[in]  msg   The message to dumps (related to the called SDK function).
 * @param[in]  s     The functions's status output.
 */
void check_status(const char *msg, int s) {
  switch (s) {
  case EEMAGINE_SDK_NOT_CONNECTED:
    fprintf(stderr, "%s: status is not connected\n", msg);
    break;
  case EEMAGINE_SDK_ALREADY_EXISTS:
    fprintf(stderr, "%s: status is already exists\n", msg);
    break;
  case EEMAGINE_SDK_NOT_FOUND:
    fprintf(stderr, "%s: status is not found\n", msg);
    break;
  case EEMAGINE_SDK_INCORRECT_VALUE:
    fprintf(stderr, "%s: status is incorrect value\n", msg);
    break;
  case EEMAGINE_SDK_INTERNAL_ERROR:
    fprintf(stderr, "%s: status is internal error\n", msg);
    break;
  case EEMAGINE_SDK_UNKNOWN:
    fprintf(stderr, "%s: status is unknown\n", msg);
    break;
  default:
    printf("%s: status ok\n", msg);
    break;
  }
}


/**
 * @brief      Calculates the bipolar electrodes' mask required by the amplifier
 *             to open a stream.
 *
 * @details    The mask is a hex format. It represents the binary array of the
 *             required channels. The mask is defined according to the number of
 *             bipolar channels requested by the user (optv[2]). For now, if the
 *             user specifies X bip channels, he has to connect the first X
 *             electrodes on the bipolar boxes.
 *
 * @param      eegodev  The eegodev
 * @param      optv     The optv
 *
 * @return     The bip mask.
 */
static unsigned long long compute_bip_mask(struct eego_eegdev* eegodev,
                                           const char* optv[]) {
  // HARD CODED --> TO CHANGE!!!
  // For now required that the user use the first n EXG channels. He cannot connect to any channels.
  unsigned long long bip_mask;

  if (atoi(optv[2]) == 1)
    bip_mask = 0x000001;
  else if (atoi(optv[2]) == 2)
    bip_mask = 0x000003;
  else if (atoi(optv[2]) == 3)
    bip_mask = 0x000007;
  else if (atoi(optv[2]) == 4)
    bip_mask = 0x00000F;
  else if (atoi(optv[2]) == 5)
    bip_mask = 0x00001F;
  else if (atoi(optv[2]) == 6)
    bip_mask = 0x00003F;
  else if (atoi(optv[2]) == 7)
    bip_mask = 0x00007F;
  else if (atoi(optv[2]) == 8)
    bip_mask = 0x0000FF;
  else if (atoi(optv[2]) == 9)
    bip_mask = 0x0001FF;
  else if (atoi(optv[2]) == 10)
    bip_mask = 0x0003FF;
  else if (atoi(optv[2]) == 11)
    bip_mask = 0x0007FF;
  else if (atoi(optv[2]) == 12)
    bip_mask = 0x000FFF;
  else if (atoi(optv[2]) == 13)
    bip_mask = 0x001FFF;
  else if (atoi(optv[2]) == 14)
    bip_mask = 0x003FFF;
  else if (atoi(optv[2]) == 15)
    bip_mask = 0x007FFF;
  else if (atoi(optv[2]) == 16)
    bip_mask = 0x00FFFF;
  else if (atoi(optv[2]) == 17)
    bip_mask = 0x01FFFF;
  else if (atoi(optv[2]) == 18)
    bip_mask = 0x03FFFF;
  else if (atoi(optv[2]) == 19)
    bip_mask = 0x07FFFF;
  else if (atoi(optv[2]) == 20)
    bip_mask = 0x0FFFFF;
  else if (atoi(optv[2]) == 21)
    bip_mask = 0x1FFFFF;
  else if (atoi(optv[2]) == 22)
    bip_mask = 0x3FFFFF;
  else if (atoi(optv[2]) == 23)
    bip_mask = 0x7FFFFF;
  else if (atoi(optv[2]) == 24)
    bip_mask = 0xFFFFFF;

  return bip_mask;
}


/**
 * @brief      Initialize the amplifier and assign it to the eegdev attribute
 *             amplifier_info.
 *
 * @param      eegodev  The eegodev.
 */
static void initialize_amplifiers(struct eego_eegdev* eegodev) {

  eemagine_sdk_amplifier_info* amplifier_info_tmp;
  amplifier_info_tmp = malloc(sizeof(eemagine_sdk_amplifier_info) * 2);

  eemagine_sdk_init();
  eegodev->amplifiers_nb =
      eemagine_sdk_get_amplifiers_info(amplifier_info_tmp, 2);
  check_status("get amplifiers info", eegodev->amplifiers_nb);
  printf("nb amplifier: %d\n", eegodev->amplifiers_nb);

  eegodev->NUM_EEG_CH = 0;
  eegodev->NUM_EXG_CH = 0;
  eegodev->NUM_TRI_CH = 0;
  eegodev->NUM_SAMPLE_COUNTER_CH = 0;

  // If two amplifiers connected, create a cascaded amplifier (for 128 channels)
  if (eegodev->amplifiers_nb == 2) {
    int* amplifier_info_tmp_id;
    amplifier_info_tmp_id = malloc(sizeof(int) * eegodev->amplifiers_nb);

    for (int i = 0; i < eegodev->amplifiers_nb; ++i) {  
      amplifier_info_tmp_id[i] = amplifier_info_tmp[i].id;
      printf("amp %i : %s\n", i, amplifier_info_tmp[i].serial);
    }

    eegodev->amplifier_info.id = eemagine_sdk_create_cascaded_amplifier(
        amplifier_info_tmp_id, eegodev->amplifiers_nb);
    check_status("create cascaded amplifier", eegodev->amplifier_info.id);
    free(amplifier_info_tmp_id);
  } 
  // If only one amplifier is connected.
  else {
    eegodev->amplifier_info = amplifier_info_tmp[0];
    printf("amp : %s\n", eegodev->amplifier_info.serial);
  }
  free(amplifier_info_tmp);
  
}


/**
 * @brief      Gets the reference electrodes' range.
 *
 * @param      eegodev  The eegodev.
 *
 * @return     The reference range.
 */
static double get_reference_range(struct eego_eegdev* eegodev) {
  double reference_range[64];
  check_status("get ref ranges",
               eemagine_sdk_get_amplifier_reference_ranges_available(
                   eegodev->amplifier_info.id, reference_range, 64));

  return reference_range[0];
}


/**
 * @brief      Gets the bipolar electrodes' range.
 *
 * @param      eegodev  The eegodev.
 *
 * @return     The bipolar range.
 */
static double get_bip_range(struct eego_eegdev* eegodev){
  double bipolar_range[64];
  check_status("get bip ranges",
               eemagine_sdk_get_amplifier_bipolar_ranges_available(
                   eegodev->amplifier_info.id, bipolar_range, 64));

  return bipolar_range[0];
}


/**
 * @brief      Gets the channel list from the opened stream.
 *
 * @details    The channels list of the stream can be different from the one of
 *             the amplifier as the user can select channels of interest.
 *
 * @param      eegodev  The eegodev
 */
static void get_channel_list(struct eego_eegdev* eegodev) {

  // gets the channels list.
  int bytes_to_allocate =
      sizeof(eemagine_sdk_channel_info) * eegodev->stream_nb_channels;
  eemagine_sdk_channel_info* channel_info_array;
  channel_info_array = malloc(bytes_to_allocate);
  eemagine_sdk_get_stream_channel_list(eegodev->streams_id, channel_info_array,
                                       bytes_to_allocate);
  printf("nb channels: %d\n", eegodev->stream_nb_channels);

  // Assign the number of EEG, EXG, COUNT and TRIG channels 
  for (int j = 0; j < eegodev->stream_nb_channels; ++j) {
    switch (channel_info_array[j].type) {
      case EEMAGINE_SDK_CHANNEL_TYPE_REFERENCE:
        eegodev->NUM_EEG_CH += 1;
        break;
      case EEMAGINE_SDK_CHANNEL_TYPE_BIPOLAR:
        eegodev->NUM_EXG_CH += 1;
        break;
      case EEMAGINE_SDK_CHANNEL_TYPE_TRIGGER:
        eegodev->NUM_TRI_CH += 1;
        break;
      case EEMAGINE_SDK_CHANNEL_TYPE_SAMPLE_COUNTER:
        eegodev->NUM_SAMPLE_COUNTER_CH += 1;
        break;
    }
  }
  eegodev->NCH = (eegodev->NUM_EEG_CH + eegodev->NUM_EXG_CH +
                  eegodev->NUM_TRI_CH + eegodev->NUM_SAMPLE_COUNTER_CH);

  free(channel_info_array);  
}

/**
 * @brief      Sets the cap's capabilities.
 *
 * @param      eegodev  The eegodev.
 * @param      optv     The optv.
 *
 * @return     Always 0.
 */
static int eego_set_capability(struct eego_eegdev* eegodev,
                               const char* optv[]) {
  struct systemcap cap = {
      .sampling_freq = atoi(optv[0]),
      .type_nch[EGD_EEG] = atoi(optv[1]),
      .type_nch[EGD_SENSOR] = atoi(optv[2]),
      .type_nch[EGD_TRIGGER] = atoi(optv[3]),
      .device_type = optv[4],
      .device_id = optv[5],
  };
  struct devmodule* dev = &eegodev->dev;

  eegodev->offset[EGD_EEG] = 0;
  eegodev->offset[EGD_SENSOR] = (eegodev->NUM_EEG_CH) * sizeof(double);
  eegodev->offset[EGD_TRIGGER] = eegodev->offset[EGD_SENSOR] + (eegodev->NUM_EXG_CH) * sizeof(double);

  dev->ci.set_cap(dev, &cap);
  dev->ci.set_input_samlen(dev, (eegodev->NCH - 1) * sizeof(double));
  return 0;
}


/**
 * @brief      Get the data and update the eegdev structure.
 *
 * @param      arg   A pointer on the eegodev structure.
 *
 * @return     Always NULL.
 */
static void* eego_read_fn(void* arg) {
  struct eego_eegdev* eegodev = arg;
  const struct core_interface* restrict ci = &eegodev->dev.ci;

  int runacq, buffer_status, bytes_to_allocate, nb_sample, nb_batch, samples_in_bytes;
  double* buffer;
  samples_in_bytes = (eegodev->stream_nb_channels -1) * sizeof(double);

  while (1) {
    usleep(100000);
    runacq = eegodev->runacq;
    if (!runacq) break;

    bytes_to_allocate = eemagine_sdk_prefetch(eegodev->streams_id);
    buffer = malloc(bytes_to_allocate);
    nb_sample = bytes_to_allocate / sizeof(double);
    buffer_status = eemagine_sdk_get_data(eegodev->streams_id, buffer, bytes_to_allocate);
    nb_batch = nb_sample / eegodev->stream_nb_channels;

    for (unsigned int j = 0; j < nb_batch; ++j) {
      // Update the eegdev structure with the new data
      if (ci->update_ringbuffer(&(eegodev->dev),
                                &buffer[j * eegodev->stream_nb_channels],
                                samples_in_bytes))
        break;
    }
    free(buffer);
  }
  return NULL;
error:
  ci->report_error(&(eegodev->dev), EIO);
  return NULL;
}


/**
 * @brief      Plugin's main
 *
 * @param      dev   A pointer on the devmodule structure.
 * @param      optv  The optv.
 *
 * @return     0 if successful, -1 otherwise.
 */
static int eego_open_device(struct devmodule* dev, const char* optv[]) {

  struct eego_eegdev* eegodev = get_eego(dev);

  unsigned long long ref_mask = 0xFFFFFFFFFFFFFFFF;
  unsigned long long bip_mask = compute_bip_mask(eegodev, optv);

  // Initialize the amplifiers
  initialize_amplifiers(eegodev);

  // Get the electrodes' range.
  double reference_range = get_reference_range(eegodev);
  double bipolar_range = get_bip_range(eegodev);  

  // Open the stream.
  eegodev->streams_id = eemagine_sdk_open_eeg_stream(
      eegodev->amplifier_info.id, atoi(optv[0]), reference_range, bipolar_range,
      ref_mask, bip_mask);
  check_status("open eeg stream", eegodev->streams_id);

  // Get the channels number.
  eegodev->stream_nb_channels =
      eemagine_sdk_get_stream_channel_count(eegodev->streams_id);
  check_status("get stream channel count", eegodev->stream_nb_channels);

  // Get channel list
  get_channel_list(eegodev);

  // Set capabilities
  eego_set_capability(eegodev, optv);

  // Launch the data acquisition's thread.
  pthread_mutex_init(&(eegodev->acqlock), NULL);
  eegodev->runacq = 1;
  int ret;
  if ((ret =
           pthread_create(&(eegodev->thread_id), NULL, eego_read_fn, eegodev)))
    goto error;

  return 0;

error:
  return -1;
}


/**
 * @brief      Close the communication with the device
 *
 * @param      dev   A pointer on the devmodule struct.
 *
 * @return     Always true
 */
static int eego_close_device(struct devmodule* dev) {
  struct eego_eegdev* eegodev = get_eego(dev);
  
  pthread_mutex_lock(&eegodev->acqlock);
  eegodev->runacq = 0;
  pthread_mutex_unlock(&eegodev->acqlock);
  check_status("close stream: ", eemagine_sdk_close_stream(eegodev->streams_id));
  check_status("close amplifier: ",eemagine_sdk_close_amplifier(eegodev->amplifier_info.id));

  pthread_join(eegodev->thread_id, NULL);
  pthread_mutex_destroy(&eegodev->acqlock);
  eemagine_sdk_exit();
  return 0;
}


/**
 * @brief      Set the channel groups parameters used by eegdev
 *
 * @param      dev   A pointer on the devmodule structure.
 * @param[in]  ngrp  The groups' number.
 * @param[in]  grp   The array of grpconf struct containing specific group's
 *                   informations.
 *
 * @return     Always 0.
 */
static int eego_set_channel_groups(struct devmodule* dev, unsigned int ngrp,
                                   const struct grpconf* grp) {
  unsigned int i, stype;
  struct selected_channels* selch;
  struct eego_eegdev* eegodev = get_eego(dev);

  if (!(selch = dev->ci.alloc_input_groups(dev, ngrp))) return -1;

  for (i = 0; i < ngrp; i++) {
    stype = grp[i].sensortype;
    // Set parameters of (eeg -> ringbuffer)
    selch[i].in_offset = eegodev->offset[stype] + grp[i].index * sizeof(double);
    selch[i].inlen = grp[i].nch * sizeof(double);
    selch[i].bsc = (stype == EGD_TRIGGER) ? 0 : 1;
    selch[i].typein = EGD_DOUBLE;
    selch[i].sc = eego_scales[grp[i].datatype];
    selch[i].typeout = grp[i].datatype;
    selch[i].iarray = grp[i].iarray;
    selch[i].arr_offset = grp[i].arr_offset;
  }

  return 0;
}


/**
 * @brief      Fill channel information for each groups
 *
 * @param[in]  dev    A pointer on the devmodule struct.
 * @param[in]  stype  The group type.
 * @param[in]  ich    The channel index.
 * @param      info   The egd_chinfo to fill.
 */
static void eego_fill_chinfo(const struct devmodule* dev, int stype,
                             unsigned int ich, struct egd_chinfo* info) {
  int t;
  struct eego_eegdev* eegodev = get_eego(dev);

  if (stype != EGD_TRIGGER) {
    info->isint = 0;
    info->dtype = EGD_DOUBLE;
    info->min.valdouble = -DBL_MAX;
    info->max.valdouble = DBL_MAX;
    if (eegodev->NUM_EEG_CH == 32)
      info->label = (stype == EGD_EEG) ? eegolabel32[ich] : sensorlabel[ich];
    else if (eegodev->NUM_EEG_CH == 64)
      info->label = (stype == EGD_EEG) ? eegolabel64[ich] : sensorlabel[ich];
    else if (eegodev->NUM_EEG_CH == 128)
      info->label = (stype == EGD_EEG) ? eegolabel128[ich] : sensorlabel[ich];
    t = 0;
  } else {
    info->isint = 0;
    info->dtype = EGD_DOUBLE;
    info->min.valdouble = -EGD_DOUBLE;
    info->max.valdouble = EGD_DOUBLE;
    info->label = trigglabel;
    t = 1;
  }
  info->unit = eegounit[t];
  info->transducter = eegotransducter[t];
}

API_EXPORTED
const struct egdi_plugin_info eegdev_plugin_info = {
    .plugin_abi = EEGDEV_PLUGIN_ABI_VERSION,
    .struct_size = sizeof(struct eego_eegdev),
    .open_device = eego_open_device,
    .close_device = eego_close_device,
    .set_channel_groups = eego_set_channel_groups,
    .fill_chinfo = eego_fill_chinfo,
    .supported_opts = eego_options};
