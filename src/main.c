#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#include "rf_scanner.h"

/* Global variables for signal handling */
static rtlsdr_dev_t *device = NULL;
static int do_exit = 0;

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
		"  -s <sample_rate> Sample rate in Hz (default: %.0f Hz)\n"
		"  -g <gain>        Gain in dB (default: %.1f dB)\n"
		"  -t <scan_time>   Time to scan each frequency in seconds (default: %d s)\n"
		"  -S <0|1>         Save signal data to file (default: %d)\n"
		"  -i <iterations>  Number of iterations (0 = infinite, default: %d)\n"
		"  -h               Show this help message\n",
		progname, DEFAULT_START_FREQ, DEFAULT_STOP_FREQ, DEFAULT_SAMPLE_RATE, 
		DEFAULT_GAIN, DEFAULT_SCAN_TIME, DEFAULT_SAVE_SIGNAL, DEFAULT_ITERATIONS);
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
	while ((opt = getopt(argc, argv, "f:F:s:g:t:S:i:h")) != -1) {
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
	int device_count, device_index = 0, r;
	
	/* Get number of devices */
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found\n");
		return -1;
	}
	
	fprintf(stderr, "Found %d device(s)\n", device_count);
	fprintf(stderr, "Using device #%d: %s\n", device_index, 
		rtlsdr_get_device_name(device_index));
	
	/* Open device */
	r = rtlsdr_open(dev, (uint32_t)device_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d\n", device_index);
		return -1;
	}
	
	/* Set sample rate */
	r = rtlsdr_set_sample_rate(*dev, sample_rate);
	if (r < 0) {
		fprintf(stderr, "Failed to set sample rate to %u Hz\n", sample_rate);
		return -1;
	}
	
	/* Set center frequency */
	r = rtlsdr_set_center_freq(*dev, frequency);
	if (r < 0) {
		fprintf(stderr, "Failed to set center frequency to %u Hz\n", frequency);
		return -1;
	}
	
	/* Set gain mode */
	if (gain == 0) {
		/* Enable automatic gain */
		r = rtlsdr_set_tuner_gain_mode(*dev, 0);
		if (r < 0) {
			fprintf(stderr, "Failed to enable automatic gain\n");
			return -1;
		}
		fprintf(stderr, "Automatic gain mode enabled\n");
	} else {
		/* Enable manual gain */
		r = rtlsdr_set_tuner_gain_mode(*dev, 1);
		if (r < 0) {
			fprintf(stderr, "Failed to enable manual gain\n");
			return -1;
		}
		
		/* Set gain */
		/* Convert dB to tenths of dB as per rtlsdr API */
		int gain_tenths = (int)(gain * 10.0);
		
		/* Find the nearest supported gain */
		int num_gains = rtlsdr_get_tuner_gains(*dev, NULL);
		if (num_gains <= 0) {
			fprintf(stderr, "Failed to get supported gain values\n");
			return -1;
		}
		
		int *gains = malloc(num_gains * sizeof(int));
		if (!gains) {
			fprintf(stderr, "Memory allocation error\n");
			return -1;
		}
		
		rtlsdr_get_tuner_gains(*dev, gains);
		
		/* Find the nearest gain */
		int nearest_gain = gains[0];
		int min_diff = abs(gain_tenths - gains[0]);
		
		for (int i = 1; i < num_gains; i++) {
			int diff = abs(gain_tenths - gains[i]);
			if (diff < min_diff) {
				min_diff = diff;
				nearest_gain = gains[i];
			}
		}
		
		free(gains);
		
		r = rtlsdr_set_tuner_gain(*dev, nearest_gain);
		if (r < 0) {
			fprintf(stderr, "Failed to set tuner gain\n");
			return -1;
		}
		
		fprintf(stderr, "Gain set to %.1f dB (nearest supported value)\n", 
			nearest_gain / 10.0);
	}
	
	/* Reset endpoint before streaming */
	rtlsdr_reset_buffer(*dev);
	
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
	
	/* Read samples */
	r = rtlsdr_read_sync(dev, buffer, buffer_size, &n_read);
	if (r < 0) {
		fprintf(stderr, "Failed to read samples\n");
		free(buffer);
		return -1;
	}
	
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
	
	/* Save FFT data as well */
	save_fft_to_file(filename, NULL, DEFAULT_FFT_SIZE, frequency, sample_rate);
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
	
	/* Register signal handlers */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGQUIT, signal_handler);
	
	/* Seed random number generator */
	srand(time(NULL));
	
	/* Main loop */
	while (!do_exit && (config.iterations == 0 || iterations_done < config.iterations)) {
		/* Pick a random frequency */
		current_freq = (uint32_t)random_frequency(config.start_freq, config.stop_freq, 
			config.sample_rate);
		
		/* Setup device */
		r = setup_device(&device, current_freq, config.sample_rate, config.gain);
		if (r < 0) {
			fprintf(stderr, "Failed to setup device\n");
			break;
		}
		
		/* Scan frequency */
		signal_detected = scan_frequency(device, current_freq, config.sample_rate, 
			config.scan_time, config.save_signal, config.fft_output_file);
		
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