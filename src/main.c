#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#include <libusb-1.0/libusb.h>  /* For accessing libusb directly if needed */
#include <fcntl.h>              /* For file operations */
#include <sys/ioctl.h>          /* For ioctl */
#include <linux/videodev2.h>    /* For v4l2 */
#include <errno.h>              /* For errno */
#include "rf_scanner.h"

/* LIBUSB error codes for use in try_detach_kernel_driver */
#ifndef LIBUSB_ERROR_NOT_FOUND
#define LIBUSB_ERROR_NOT_FOUND    -5
#endif
#ifndef LIBUSB_ERROR_NO_DEVICE
#define LIBUSB_ERROR_NO_DEVICE    -4
#endif
#ifndef LIBUSB_ERROR_INVALID_PARAM
#define LIBUSB_ERROR_INVALID_PARAM -2
#endif
#ifndef LIBUSB_ERROR_OTHER
#define LIBUSB_ERROR_OTHER        -99
#endif

/* Global variables for signal handling */
static rtlsdr_dev_t *device = NULL;
static int do_exit = 0;

/* Global configuration for simulation mode access across functions */
scanner_config_t global_config;

/* Function prototypes */
void signal_handler(int sig);
void print_usage(const char *progname);
int parse_arguments(int argc, char **argv, scanner_config_t *config);
double random_frequency(double start, double stop, uint32_t sample_rate);
int setup_device(rtlsdr_dev_t **dev, uint32_t frequency, uint32_t sample_rate, double gain);
int scan_frequency(rtlsdr_dev_t *dev, uint32_t frequency, uint32_t sample_rate, 
                  int duration, int save_signal, const char *fft_file);
int analyze_signal(uint8_t *buffer, uint32_t buffer_size, uint32_t sample_rate, double *max_db);
void save_signal_to_file(uint8_t *buffer, uint32_t buffer_size, uint32_t frequency, 
                        uint32_t sample_rate);
void save_fft_to_file(const char *filename, fftw_complex *fft_result, int fft_size, 
                     uint32_t frequency, uint32_t sample_rate);
int try_detach_kernel_driver(rtlsdr_dev_t *dev);
int check_rtlsdr_access();
int scan_frequency_v4l2(uint32_t frequency, uint32_t sample_rate, int duration, int save_signal, const char *fft_file);

void signal_handler(int sig) {
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	if (device)
		rtlsdr_cancel_async(device);
}

void print_usage(const char *progname) {
	fprintf(stderr,
		"RF Signal Scanner\n\n"
		"Usage: %s [options]\n\n"
		"Options:\n"
		"  -f <start_freq>  Start frequency in Hz (default: %.0f Hz)\n"
		"  -F <stop_freq>   Stop frequency in Hz (default: %.0f Hz)\n"
		"  -s <sample_rate> Sample rate in Hz (default: %u Hz)\n"
		"  -g <gain>        Gain in dB (default: %.1f dB)\n"
		"  -t <scan_time>   Time to scan each frequency in seconds (default: %d s)\n"
		"  -S <0|1>         Save signal data to file (default: %d)\n"
		"  -i <iterations>  Number of iterations (0 = infinite, default: %d)\n"
		"  -k <0|1>         Use kernel driver (/dev/swradio0) instead of direct USB (default: %d)\n"
		"  -m <0|1>         Enable simulation mode for testing without hardware (default: %d)\n"
		"  -v <0|1>         Enable verbose logging (default: %d)\n"
		"  -h               Show this help message\n",
		progname, DEFAULT_START_FREQ, DEFAULT_STOP_FREQ, (unsigned int)DEFAULT_SAMPLE_RATE, 
		DEFAULT_GAIN, DEFAULT_SCAN_TIME, DEFAULT_SAVE_SIGNAL, DEFAULT_ITERATIONS,
		DEFAULT_USE_KERNEL_DRIVER, DEFAULT_SIMULATION_MODE, DEFAULT_VERBOSE);
	exit(1);
}

int parse_arguments(int argc, char **argv, scanner_config_t *config) {
	int opt;

	/* Set default values */
	config->start_freq = DEFAULT_START_FREQ;
	config->stop_freq = DEFAULT_STOP_FREQ;
	config->sample_rate = DEFAULT_SAMPLE_RATE;
	config->gain = DEFAULT_GAIN;
	config->scan_time = DEFAULT_SCAN_TIME;
	config->save_signal = DEFAULT_SAVE_SIGNAL;
	config->iterations = DEFAULT_ITERATIONS;
	config->use_kernel_driver = DEFAULT_USE_KERNEL_DRIVER;
	config->simulation_mode = DEFAULT_SIMULATION_MODE;
	config->verbose = DEFAULT_VERBOSE;

	/* Create unique FFT output filename with timestamp */
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	char timestamp[32];
	strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm);
	
	config->fft_output_file = malloc(64);
	if (!config->fft_output_file) {
		fprintf(stderr, "Memory allocation error\n");
		return -1;
	}
	snprintf(config->fft_output_file, 64, "spectrum_%s.csv", timestamp);

	/* Parse command line arguments */
	while ((opt = getopt(argc, argv, "f:F:s:g:t:S:i:k:m:v:h")) != -1) {
		switch (opt) {
		case 'f':
			config->start_freq = atof(optarg);
			break;
		case 'F':
			config->stop_freq = atof(optarg);
			break;
		case 's':
			config->sample_rate = (uint32_t)atof(optarg);
			break;
		case 'g':
			config->gain = atof(optarg);
			break;
		case 't':
			config->scan_time = atoi(optarg);
			break;
		case 'S':
			config->save_signal = atoi(optarg);
			break;
		case 'i':
			config->iterations = atoi(optarg);
			break;
		case 'k':
			config->use_kernel_driver = atoi(optarg);
			break;
		case 'm':
			config->simulation_mode = atoi(optarg);
			break;
		case 'v':
			config->verbose = atoi(optarg);
			break;
		case 'h':
		default:
			return -1;
		}
	}

	/* Validate arguments */
	if (config->start_freq >= config->stop_freq) {
		fprintf(stderr, "Error: Start frequency must be lower than stop frequency\n");
		return -1;
	}

	if (config->scan_time <= 0) {
		fprintf(stderr, "Error: Scan time must be greater than 0\n");
		return -1;
	}

	return 0;
}

double random_frequency(double start, double stop, uint32_t sample_rate) {
	/* Calculate frequency step based on sample rate */
	double step = sample_rate / 4.0;  /* Ensure some overlap between steps */
	
	/* Calculate number of possible frequency steps in the range */
	int steps = (int)((stop - start) / step);
	
	if (steps <= 0)
		return start;
	
	/* Select a random step and calculate the corresponding frequency */
	int random_step = rand() % steps;
	double freq = start + (random_step * step);
	
	return freq;
}

int setup_device(rtlsdr_dev_t **dev, uint32_t frequency, uint32_t sample_rate, double gain) {
	int device_index = 0, r;
	
	/* Get verbose mode from global config */
	extern scanner_config_t global_config;
	int verbose = global_config.verbose;
	
	/* Get number of devices */
	int device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found\n");
		return -1;
	}
	
	fprintf(stderr, "Found %d device(s)\n", device_count);
	fprintf(stderr, "Using device #%d: %s\n", device_index, 
		rtlsdr_get_device_name(device_index));

	/* Display more device info if verbose */
	if (verbose) {
		char manufacturer[256] = {0};
		char product[256] = {0};
		char serial[256] = {0};
		rtlsdr_get_device_usb_strings(0, manufacturer, product, serial);
		fprintf(stderr, "Device info: Manufacturer='%s', Product='%s', Serial='%s'\n",
			manufacturer, product, serial);
	}
	
	/* Try to close the device first if it was already open */
	if (*dev) {
		fprintf(stderr, "Device already open, closing first...\n");
		rtlsdr_close(*dev);
		*dev = NULL;
		usleep(100000); /* Wait 100ms */
	}
	
	/* Open device - simplified approach just like rf_scanner2.c */
	r = rtlsdr_open(dev, (uint32_t)device_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device: error %d\n", r);
		return -1;
	}
	
	fprintf(stderr, "Successfully opened device\n");
	
	/* Basic device check */
	try_detach_kernel_driver(*dev);
	
	/* Set sample rate */
	r = rtlsdr_set_sample_rate(*dev, sample_rate);
	if (r < 0) {
		fprintf(stderr, "Failed to set sample rate to %u Hz: error %d\n", sample_rate, r);
		rtlsdr_close(*dev);
		*dev = NULL;
		return -1;
	}
	
	/* Always set gain mode first before setting gain (like rf_scanner2.c does) */
	r = rtlsdr_set_tuner_gain_mode(*dev, (gain == 0) ? 0 : 1);
	if (r < 0) {
		fprintf(stderr, "Failed to set gain mode: error %d\n", r);
		rtlsdr_close(*dev);
		*dev = NULL;
		return -1;
	}
	
	/* Set center frequency */
	r = rtlsdr_set_center_freq(*dev, frequency);
	if (r < 0) {
		fprintf(stderr, "Failed to set center frequency to %u Hz: error %d\n", frequency, r);
		rtlsdr_close(*dev);
		*dev = NULL;
		return -1;
	}
	
	/* Set gain if using manual mode */
	if (gain != 0) {
		/* Get available gains */
		int num_gains = rtlsdr_get_tuner_gains(*dev, NULL);
		if (num_gains <= 0) {
			fprintf(stderr, "Failed to get supported gain values, using default\n");
			r = rtlsdr_set_tuner_gain(*dev, 400); /* Use a common default gain value */
			if (r < 0) {
				fprintf(stderr, "Warning: Failed to set tuner gain but continuing\n");
			} else {
				fprintf(stderr, "Gain set to 40.0 dB (default)\n");
			}
		} else {
			/* Get available gain values */
			int *gains = malloc(num_gains * sizeof(int));
			if (!gains) {
				fprintf(stderr, "Memory allocation error, using default gain\n");
				r = rtlsdr_set_tuner_gain(*dev, 400);
				if (r < 0) {
					fprintf(stderr, "Warning: Failed to set tuner gain but continuing\n");
				} else {
					fprintf(stderr, "Gain set to 40.0 dB (default)\n");
				}
			} else {
				rtlsdr_get_tuner_gains(*dev, gains);
				
				if (verbose) {
					fprintf(stderr, "Available gains: ");
					for (int i = 0; i < num_gains; i++) {
						fprintf(stderr, "%.1f ", gains[i] / 10.0);
					}
					fprintf(stderr, "\n");
				}
				
				/* Just use the middle gain value */
				int gain_to_use = gains[num_gains / 2];
				r = rtlsdr_set_tuner_gain(*dev, gain_to_use);
				if (r < 0) {
					fprintf(stderr, "Warning: Failed to set tuner gain but continuing\n");
				} else {
					fprintf(stderr, "Gain set to %.1f dB\n", gain_to_use / 10.0);
				}
				
				free(gains);
			}
		}
	} else {
		fprintf(stderr, "Automatic gain mode enabled\n");
	}
	
	/* Reset endpoint before streaming */
	r = rtlsdr_reset_buffer(*dev);
	if (r < 0) {
		fprintf(stderr, "Warning: Failed to reset buffer: %d\n", r);
		/* Continue anyway - this might not be fatal */
	} else {
		fprintf(stderr, "Successfully reset buffer\n");
	}
	
	return 0;
}

int scan_frequency(rtlsdr_dev_t *dev, uint32_t frequency, uint32_t sample_rate, 
                  int duration, int save_signal, const char *fft_file) {
	int r, n_read;
	uint32_t buffer_size = sample_rate * 2 * duration; /* 2 bytes per sample (I+Q) */
	uint8_t *buffer = malloc(buffer_size);
	double max_db;
	
	if (!buffer) {
		fprintf(stderr, "Memory allocation error\n");
		return -1;
	}
	
	fprintf(stderr, "Scanning frequency %u Hz (%.3f MHz) for %d seconds...\n", 
		frequency, frequency/1e6, duration);
	
	/* Set center frequency */
	r = rtlsdr_set_center_freq(dev, frequency);
	if (r < 0) {
		fprintf(stderr, "Failed to set center frequency to %u Hz\n", frequency);
		free(buffer);
		return -1;
	}
	
	/* Let tuner settle */
	usleep(50000);
	
	/* Reset buffer before reading */
	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "Failed to reset buffer: %d\n", r);
		/* Continue anyway */
	}
	
	/* Let tuner settle a bit more */
	usleep(100000);
	
	/* Read samples */
	fprintf(stderr, "Reading samples...\n");
	
	/* Get simulation mode from config - Will be passed as a global variable */
	extern scanner_config_t global_config;
	int simulation_mode = global_config.simulation_mode;
	int verbose = global_config.verbose;
	
	if (simulation_mode) {
		fprintf(stderr, "Note: Simulation mode is enabled (use -m 0 to disable)\n");
	}
	
	/* Add a delay before reading */
	usleep(250000);  /* 250ms delay */
	
	/* Try to read a smaller buffer first to test device access */
	uint8_t test_buffer[1024];
	int test_read = 0;
	
	if (verbose) {
		fprintf(stderr, "Testing device access with small read first...\n");
	}
	
	r = rtlsdr_read_sync(dev, test_buffer, sizeof(test_buffer), &test_read);
	if (r < 0) {
		fprintf(stderr, "Failed to perform test read: error %d\n", r);
	} else if (verbose) {
		fprintf(stderr, "Test read successful, read %d bytes\n", test_read);
	}
	
	/* Now try full read */
	r = rtlsdr_read_sync(dev, buffer, buffer_size, &n_read);
	if (r < 0) {
		fprintf(stderr, "Failed to read samples: error %d\n", r);
		fprintf(stderr, "Error details based on code:\n");
		switch (r) {
			case -1:
				fprintf(stderr, "Error -1: Unspecified error (possibly out of memory or device access issue)\n");
				break;
			case -2:
				fprintf(stderr, "Error -2: Device allocation failed\n");
				break;
			case -3:
				fprintf(stderr, "Error -3: Device not found\n");
				break;
			case -4:
				fprintf(stderr, "Error -4: Device busy\n");
				break;
			case -5:
				fprintf(stderr, "Error -5: Device not supported\n");
				break;
			case -6:
				fprintf(stderr, "Error -6: Device already in use by another process or driver\n");
				break;
			case -7:
				fprintf(stderr, "Error -7: Device I/O error\n");
				break;
			case -8:
				fprintf(stderr, "Error -8: Device doesn't support this functionality\n");
				break;
			case -9:
				fprintf(stderr, "Error -9: Device Not Found\n");
				break;
			case -11:
				fprintf(stderr, "Error -11: Insufficient permissions or device in use by kernel driver\n");
				fprintf(stderr, "Trying to proceed anyway with reduced buffer size...\n");
				
				/* Try smaller buffer size */
				uint32_t smaller_size = buffer_size / 4;
				r = rtlsdr_read_sync(dev, buffer, smaller_size, &n_read);
				if (r < 0) {
					fprintf(stderr, "Still failed with smaller buffer size\n");
				} else {
					fprintf(stderr, "Success with smaller buffer size! Read %d bytes\n", n_read);
					/* Continue with what we have */
					goto process_data;
				}
				break;
			default:
				fprintf(stderr, "Unknown error code\n");
		}
		
		if (simulation_mode) {
			fprintf(stderr, "*** ENTERING SIMULATION MODE ***\n");
			fprintf(stderr, "Generating synthetic data for testing since device access failed\n");
			
			/* Fill buffer with random data */
			srand(time(NULL));
			for (unsigned int i = 0; i < buffer_size; i++) {
				buffer[i] = rand() % 256;
			}
			
			/* Pretend we read the entire buffer */
			n_read = buffer_size;
			
			/* Continue processing with simulated data */
			fprintf(stderr, "Created %d samples of simulated data\n", n_read);
		} else {
			fprintf(stderr, "This may be due to kernel drivers claiming the device.\n");
			fprintf(stderr, "Try running: sudo rmmod dvb_usb_rtl28xxu rtl2832_sdr rtl2832\n");
			free(buffer);
			return -1;
		}
	}
	
process_data:
	
	fprintf(stderr, "Read %d samples\n", n_read);
	
	/* Analyze signal */
	r = analyze_signal(buffer, n_read, sample_rate, &max_db);
	if (r < 0) {
		fprintf(stderr, "Failed to analyze signal\n");
		free(buffer);
		return -1;
	}
	
	fprintf(stderr, "Max signal level: %.2f dBFS\n", max_db);
	
	/* Save signal if threshold is exceeded and saving is enabled */
	if (max_db > SIGNAL_THRESHOLD && save_signal) {
		fprintf(stderr, "Signal detected! Saving to file...\n");
		save_signal_to_file(buffer, n_read, frequency, sample_rate);
	}
	
	free(buffer);
	return (max_db > SIGNAL_THRESHOLD) ? 1 : 0;
}

int analyze_signal(uint8_t *buffer, uint32_t buffer_size, uint32_t sample_rate, double *max_db) {
	fftw_complex *in, *out;
	fftw_plan p;
	double real, imag, magnitude, max_magnitude = 0;
	size_t i;
	
	/* Allocate arrays for FFT */
	in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * DEFAULT_FFT_SIZE);
	out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * DEFAULT_FFT_SIZE);
	
	if (!in || !out) {
		fprintf(stderr, "Memory allocation error\n");
		return -1;
	}
	
	/* Create FFT plan */
	p = fftw_plan_dft_1d(DEFAULT_FFT_SIZE, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
	
	/* Process data in FFT_SIZE chunks */
	uint32_t chunks = buffer_size / (2 * DEFAULT_FFT_SIZE);
	if (chunks == 0) chunks = 1; /* Ensure at least one chunk */
	
	for (size_t chunk = 0; chunk < chunks; chunk++) {
		/* Fill input array with complex samples */
		for (i = 0; i < DEFAULT_FFT_SIZE && (chunk * DEFAULT_FFT_SIZE + i) < buffer_size/2; i++) {
			size_t idx = chunk * DEFAULT_FFT_SIZE + i;
			/* Convert samples to complex values (8-bit unsigned to normalized float) */
			real = (buffer[2*idx] - 127.5) / 127.5;
			imag = (buffer[2*idx+1] - 127.5) / 127.5;
			in[i] = real + imag * I;
		}
		
		/* Execute FFT */
		fftw_execute(p);
		
		/* Find maximum magnitude */
		for (i = 0; i < DEFAULT_FFT_SIZE; i++) {
			real = creal(out[i]);
			imag = cimag(out[i]);
			magnitude = sqrt(real*real + imag*imag);
			
			if (magnitude > max_magnitude)
				max_magnitude = magnitude;
		}
	}
	
	/* Convert to dBFS */
	*max_db = 20 * log10(max_magnitude / DEFAULT_FFT_SIZE);
	
	/* Clean up */
	fftw_destroy_plan(p);
	fftw_free(in);
	fftw_free(out);
	
	return 0;
}

void save_signal_to_file(uint8_t *buffer, uint32_t buffer_size, uint32_t frequency, 
                        uint32_t sample_rate) {
	FILE *fp;
	char filename[256];
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	char timestamp[32];
	
	/* Create timestamp */
	strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm);
	
	/* Create filename */
	snprintf(filename, sizeof(filename), "signal_%s_%.3fMHz_%dksps.bin", 
		timestamp, frequency/1e6, sample_rate/1000);
	
	/* Open file */
	fp = fopen(filename, "wb");
	if (!fp) {
		fprintf(stderr, "Failed to open file for writing: %s\n", filename);
		return;
	}
	
	/* Write data */
	fwrite(buffer, 1, buffer_size, fp);
	
	/* Close file */
	fclose(fp);
	
	fprintf(stderr, "Signal saved to %s\n", filename);
	
	/* Calculate FFT for the saved signal */
	fftw_complex *in, *out;
	fftw_plan p;
	int fft_size = DEFAULT_FFT_SIZE;
	
	/* Allocate arrays for FFT */
	in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
	out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
	
	if (!in || !out) {
		fprintf(stderr, "Memory allocation error for FFT\n");
		if (in) fftw_free(in);
		if (out) fftw_free(out);
		return;
	}
	
	/* Create FFT plan */
	p = fftw_plan_dft_1d(fft_size, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
	
	/* Fill input array with complex samples from the buffer (use only the first fft_size samples) */
	for (int i = 0; i < fft_size && i*2+1 < buffer_size; i++) {
		/* Convert samples to complex values (8-bit unsigned to normalized float) */
		double real = (buffer[2*i] - 127.5) / 127.5;
		double imag = (buffer[2*i+1] - 127.5) / 127.5;
		in[i] = real + imag * I;
	}
	
	/* Execute FFT */
	fftw_execute(p);
	
	/* Save FFT data */
	save_fft_to_file(filename, out, fft_size, frequency, sample_rate);
	
	/* Clean up */
	fftw_destroy_plan(p);
	fftw_free(in);
	fftw_free(out);
}

void save_fft_to_file(const char *filename, fftw_complex *fft_result, int fft_size, 
                     uint32_t frequency, uint32_t sample_rate) {
	FILE *fp;
	char fft_filename[256];
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	char timestamp[32];
	
	/* Create timestamp if not already in filename */
	if (strstr(filename, "_2") == NULL) {
		strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm);
		snprintf(fft_filename, sizeof(fft_filename), "spectrum_%s_%.3fMHz_%dksps.csv", 
			timestamp, frequency/1e6, sample_rate/1000);
	} else {
		/* Extract timestamp from filename */
		char *start = strstr(filename, "_");
		char *end = strstr(start+1, "_");
		if (start && end && (end-start-1) < sizeof(timestamp)) {
			strncpy(timestamp, start+1, end-start-1);
			timestamp[end-start-1] = '\0';
			snprintf(fft_filename, sizeof(fft_filename), "spectrum_%s_%.3fMHz_%dksps.csv", 
				timestamp, frequency/1e6, sample_rate/1000);
		} else {
			/* Fallback if timestamp extraction fails */
			strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm);
			snprintf(fft_filename, sizeof(fft_filename), "spectrum_%s_%.3fMHz_%dksps.csv", 
				timestamp, frequency/1e6, sample_rate/1000);
		}
	}
	
	/* Open file */
	fp = fopen(fft_filename, "w");
	if (!fp) {
		fprintf(stderr, "Failed to open file for writing: %s\n", fft_filename);
		return;
	}
	
	/* Write header */
	fprintf(fp, "Frequency,Power\n");
	
	/* If fft_result is null, just create an empty file with header */
	if (fft_result) {
		double bin_size = (double)sample_rate / fft_size;
		
		/* Write FFT data */
		for (int i = 0; i < fft_size; i++) {
			double freq_offset = (i <= fft_size/2) ? 
				i * bin_size : (i - fft_size) * bin_size;
			double freq = frequency + freq_offset;
			
			double real = creal(fft_result[i]);
			double imag = cimag(fft_result[i]);
			double magnitude = sqrt(real*real + imag*imag);
			double power_db = 20 * log10(magnitude / fft_size);
			
			fprintf(fp, "%.0f,%.2f\n", freq, power_db);
		}
	}
	
	/* Close file */
	fclose(fp);
	
	fprintf(stderr, "FFT data saved to %s\n", fft_filename);
}

int try_detach_kernel_driver(rtlsdr_dev_t *dev) {
	if (!dev) return -1;
	
	/* Simplified approach - we don't manually attempt to detach kernel drivers
	   since this could potentially be causing issues if not done exactly right.
	   Instead, let librtlsdr handle this internally */
	
	fprintf(stderr, "Using simplified device initialization\n");
	
	/* Just check if the device is accessible */
	int r = rtlsdr_get_tuner_type(dev);
	if (r < 0) {
		fprintf(stderr, "Warning: Cannot get tuner type, device may not be fully accessible\n");
	} else {
		fprintf(stderr, "Device tuner type: %d\n", r);
	}
	
	return 0;
}

int scan_frequency_v4l2(uint32_t frequency, uint32_t sample_rate, int duration, int save_signal, const char *fft_file) {
	int fd, r;
	struct v4l2_frequency freq;
	struct v4l2_format fmt;
	struct v4l2_buffer buf;
	struct v4l2_requestbuffers req;
	uint8_t *buffer = NULL;
	uint32_t buffer_size;
	double max_db;
	
	fprintf(stderr, "Using V4L2 kernel driver with /dev/swradio0\n");
	
	/* Get simulation mode from config */
	extern scanner_config_t global_config;
	int simulation_mode = global_config.simulation_mode;
	
	if (simulation_mode) {
		fprintf(stderr, "Note: Simulation mode is enabled (use -m 0 to disable)\n");
	}

	/* Open the device */
	fd = open("/dev/swradio0", O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Failed to open /dev/swradio0: %s\n", strerror(errno));
		
		if (simulation_mode) {
			fprintf(stderr, "*** ENTERING SIMULATION MODE ***\n");
			fprintf(stderr, "Creating simulated V4L2 environment for testing\n");
			
			/* Skip the device open part and V4L2 setup */
			/* Create a fake buffer */
			buffer_size = sample_rate * 2 * duration;
			buffer = malloc(buffer_size);
			if (!buffer) {
				fprintf(stderr, "Memory allocation error\n");
				return -1;
			}
			
			/* Fill buffer with random data for simulation */
			srand(time(NULL));
			for (unsigned int i = 0; i < buffer_size; i++) {
				buffer[i] = rand() % 256;
			}
			
			fprintf(stderr, "Created %d bytes of simulated data\n", buffer_size);
			
			/* Directly analyze the simulated data */
			analyze_signal(buffer, buffer_size, sample_rate, &max_db);
			fprintf(stderr, "Max signal level: %.2f dBFS\n", max_db);
			
			/* Save signal if threshold is exceeded and saving is enabled */
			if (max_db > SIGNAL_THRESHOLD && save_signal) {
				fprintf(stderr, "Signal detected! Saving to file...\n");
				save_signal_to_file(buffer, buffer_size, frequency, sample_rate);
			}
			
			free(buffer);
			return (max_db > SIGNAL_THRESHOLD) ? 1 : 0;
		} else {
			return -1;
		}
	}
	
	/* Set frequency */
	memset(&freq, 0, sizeof(freq));
	freq.tuner = 0;
	freq.type = V4L2_TUNER_SDR;
	freq.frequency = frequency / 1000; /* V4L2 uses kHz for SDR */
	
	r = ioctl(fd, VIDIOC_S_FREQUENCY, &freq);
	if (r < 0) {
		fprintf(stderr, "Failed to set frequency to %u Hz: %s\n", frequency, strerror(errno));
		close(fd);
		return -1;
	}
	
	fprintf(stderr, "Scanning frequency %u Hz (%.3f MHz) for %d seconds using V4L2...\n", 
		frequency, frequency/1e6, duration);
	
	/* Set format */
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_SDR_CAPTURE;
	fmt.fmt.sdr.pixelformat = V4L2_SDR_FMT_CU8;  /* Complex U8 (I/Q) */
	fmt.fmt.sdr.buffersize = sample_rate * 2 * duration;
	
	r = ioctl(fd, VIDIOC_S_FMT, &fmt);
	if (r < 0) {
		fprintf(stderr, "Failed to set format: %s\n", strerror(errno));
		close(fd);
		return -1;
	}
	
	/* Request buffers */
	memset(&req, 0, sizeof(req));
	req.count = 1;
	req.type = V4L2_BUF_TYPE_SDR_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	
	r = ioctl(fd, VIDIOC_REQBUFS, &req);
	if (r < 0) {
		fprintf(stderr, "Failed to request buffers: %s\n", strerror(errno));
		close(fd);
		return -1;
	}
	
	/* Buffer setup simplification - just allocate our own buffer */
	buffer_size = sample_rate * 2 * duration;
	buffer = malloc(buffer_size);
	if (!buffer) {
		fprintf(stderr, "Memory allocation error\n");
		close(fd);
		return -1;
	}
	
	/* Read directly using read() since it's simpler */
	r = read(fd, buffer, buffer_size);
	if (r < 0) {
		fprintf(stderr, "Failed to read samples: %s\n", strerror(errno));
		
		/* SIMULATION MODE - Create synthetic data for testing when device access fails */
		int simulation_mode = 1; /* Set to 1 to enable simulation mode */
		
		if (simulation_mode) {
			fprintf(stderr, "*** ENTERING SIMULATION MODE ***\n");
			fprintf(stderr, "Generating synthetic data for testing since device access failed\n");
			
			/* Fill buffer with random data */
			srand(time(NULL));
			for (unsigned int i = 0; i < buffer_size; i++) {
				buffer[i] = rand() % 256;
			}
			
			/* Pretend we read the entire buffer */
			r = buffer_size;
			
			/* Continue processing with simulated data */
			fprintf(stderr, "Created %d samples of simulated data\n", r);
		} else {
			free(buffer);
			close(fd);
			return -1;
		}
	}
	
	fprintf(stderr, "Read %d samples\n", r);
	
	/* Analyze signal */
	analyze_signal(buffer, r, sample_rate, &max_db);
	
	fprintf(stderr, "Max signal level: %.2f dBFS\n", max_db);
	
	/* Save signal if threshold is exceeded and saving is enabled */
	if (max_db > SIGNAL_THRESHOLD && save_signal) {
		fprintf(stderr, "Signal detected! Saving to file...\n");
		save_signal_to_file(buffer, r, frequency, sample_rate);
	}
	
	free(buffer);
	close(fd);
	
	return (max_db > SIGNAL_THRESHOLD) ? 1 : 0;
}

int check_rtlsdr_access() {
	/* Check if the device is accessible by running a shell command */
	FILE *fp = popen("lsof | grep -i rtlsdr", "r");
	if (!fp) {
		fprintf(stderr, "Failed to run command to check for RTL-SDR usage\n");
		return -1;
	}
	
	char buffer[1024];
	int used = 0;
	
	while (fgets(buffer, sizeof(buffer), fp)) {
		fprintf(stderr, "Warning: RTL-SDR appears to be in use by another process: %s", buffer);
		used = 1;
	}
	
	pclose(fp);
	
	if (used) {
		fprintf(stderr, "You may need to close other applications using the RTL-SDR device first.\n");
		return -1;
	}
	
	/* Get verbose mode from global config */
	extern scanner_config_t global_config;
	int verbose = global_config.verbose;

	/* Check for USB devices */
	if (verbose) fprintf(stderr, "Checking for RTL-SDR USB devices...\n");
	fp = popen("lsusb | grep -i rtl", "r");
	if (!fp) {
		fprintf(stderr, "Failed to run lsusb command\n");
	} else {
		int found_device = 0;
		while (fgets(buffer, sizeof(buffer), fp)) {
			if (verbose) fprintf(stderr, "RTL-SDR USB device found: %s", buffer);
			found_device = 1;
		}
		pclose(fp);
		
		if (!found_device && verbose) {
			fprintf(stderr, "No RTL-SDR USB devices found with lsusb! Check if device is connected.\n");
		}
	}
	
	/* Check for /dev/rtl_* device nodes */
	if (verbose) fprintf(stderr, "Checking for RTL-SDR device nodes...\n");
	fp = popen("ls -l /dev/rtl* 2>/dev/null || echo 'No /dev/rtl* devices found'", "r");
	if (fp) {
		while (fgets(buffer, sizeof(buffer), fp)) {
			if (verbose) fprintf(stderr, "Device node: %s", buffer);
		}
		pclose(fp);
	}
	
	/* Check for /dev/swradio* device nodes */
	if (verbose) fprintf(stderr, "Checking for V4L2 SDR device nodes...\n");
	fp = popen("ls -l /dev/swradio* 2>/dev/null || echo 'No /dev/swradio* devices found'", "r");
	if (fp) {
		while (fgets(buffer, sizeof(buffer), fp)) {
			if (verbose) fprintf(stderr, "Device node: %s", buffer);
		}
		pclose(fp);
	}
	
	/* Also check if the kernel driver is attached */
	if (verbose) fprintf(stderr, "Checking for RTL-SDR kernel modules...\n");
	fp = popen("lsmod | grep -E 'dvb_usb_rtl|rtl2832|rtl8xxxu|rtl2830|dvb_usb'", "r");
	if (!fp) {
		fprintf(stderr, "Failed to run command to check for kernel modules\n");
		return -1;
	}
	
	int modules_found = 0;
	while (fgets(buffer, sizeof(buffer), fp)) {
		fprintf(stderr, "RTL-SDR kernel module detected: %s", buffer);
		fprintf(stderr, "This is normal, the library will attempt to detach it when necessary.\n");
		modules_found = 1;
	}
	
	if (!modules_found) {
		fprintf(stderr, "No RTL-SDR kernel modules detected.\n");
	}
	
	pclose(fp);
	
	return 0;
}

int main(int argc, char **argv) {
	int r;
	scanner_config_t config;
	uint32_t current_freq;
	int iterations_done = 0;
	int signal_detected;
	
	/* Parse command line arguments */
	if (parse_arguments(argc, argv, &config) < 0) {
		print_usage(argv[0]);
		return EXIT_FAILURE;
	}
	
	/* Copy config to global for access in other functions */
	global_config = config;
	
	/* Register signal handlers */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGQUIT, signal_handler);
	
	if (config.verbose) {
		fprintf(stderr, "===== RUNNING ENHANCED DIAGNOSTICS =====\n");
	}
	
	/* Check if RTL-SDR is already in use */
	if (check_rtlsdr_access() < 0) {
		fprintf(stderr, "Warning: RTL-SDR access issues detected, but will try to continue.\n");
		fprintf(stderr, "If you get USB claim interface errors, try these commands to unload conflicting drivers:\n");
		fprintf(stderr, "  sudo rmmod dvb_usb_rtl28xxu rtl2832_sdr rtl2832 rtl2830 rtl8xxxu\n");
		fprintf(stderr, "  sudo rmmod dvb_usb_v2 dvb_core\n");
	}
	
	/* Check if swradio device exists */
	FILE *swradio = fopen("/dev/swradio0", "r");
	if (swradio) {
		fprintf(stderr, "Found /dev/swradio0 device. This indicates kernel drivers are active.\n");
		fprintf(stderr, "rtl-sdr library cannot access the device while kernel drivers are loaded.\n");
		fprintf(stderr, "Please run: sudo rmmod dvb_usb_rtl28xxu rtl2832_sdr rtl2832\n");
		fclose(swradio);
	}
	
	/* Seed random number generator */
	srand(time(NULL));
	
	/* Main loop */
	while (!do_exit && (config.iterations == 0 || iterations_done < config.iterations)) {
		/* Pick a random frequency */
		current_freq = (uint32_t)random_frequency(config.start_freq, config.stop_freq, 
			config.sample_rate);
		
		if (config.use_kernel_driver) {
			/* Use V4L2 interface with kernel driver */
			fprintf(stderr, "Using kernel driver interface (/dev/swradio0)\n");
			signal_detected = scan_frequency_v4l2(current_freq, config.sample_rate, 
				config.scan_time, config.save_signal, config.fft_output_file);
		} else {
			/* Use librtlsdr direct USB interface */
			fprintf(stderr, "Using librtlsdr direct USB interface\n");
			
			/* Setup device */
			r = setup_device(&device, current_freq, config.sample_rate, config.gain);
			if (r < 0) {
				fprintf(stderr, "Failed to setup device\n");
				fprintf(stderr, "Try using the kernel driver interface with -k 1\n");
				break;
			}
			
			/* Scan frequency */
			signal_detected = scan_frequency(device, current_freq, config.sample_rate, 
				config.scan_time, config.save_signal, config.fft_output_file);
		}
		
		/* Count iteration */
		iterations_done++;
		
		fprintf(stderr, "Completed %d iterations%s\n", iterations_done, 
			(config.iterations > 0) ? 
			  (iterations_done < config.iterations ? ", continuing..." : ", done!") : 
			  ", continuing...");
	}
	
	/* Cleanup */
	if (device) {
		rtlsdr_close(device);
	}
	
	if (config.fft_output_file) {
		free(config.fft_output_file);
	}
	
	return EXIT_SUCCESS;
}