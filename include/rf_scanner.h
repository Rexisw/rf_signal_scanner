#ifndef RF_SCANNER_H
#define RF_SCANNER_H

#include <stdint.h>
#include <rtl-sdr.h>
#include <fftw3.h>
#include <libusb-1.0/libusb.h>

#define DEFAULT_START_FREQ      88e6    /* 88 MHz */
#define DEFAULT_STOP_FREQ       108e6   /* 108 MHz */
#define DEFAULT_SAMPLE_RATE     1024000   /* 1.024 MHz */
#define DEFAULT_GAIN            40.0    /* dB */
#define DEFAULT_SCAN_TIME       10      /* seconds */
#define DEFAULT_SAVE_SIGNAL     1       /* 1 = enabled, 0 = disabled */
#define DEFAULT_ITERATIONS      0       /* 0 = run indefinitely */
#define DEFAULT_FFT_SIZE        1024    /* FFT size for spectrum analysis */
#define SIGNAL_THRESHOLD        -35.0   /* dBFS threshold for signal detection */
#define DEFAULT_USE_KERNEL_DRIVER  0    /* 0 = use librtlsdr, 1 = use kernel driver */
#define DEFAULT_SIMULATION_MODE 1       /* 1 = enable simulation, 0 = disable */
#define DEFAULT_VERBOSE         0       /* 1 = enable verbose logging, 0 = disable */

/* Configuration structure */
typedef struct {
	double start_freq;
	double stop_freq;
	uint32_t sample_rate;
	double gain;
	int scan_time;
	int save_signal;
	int iterations;
	char *fft_output_file;
	int use_kernel_driver;  /* 0 = use librtlsdr, 1 = use kernel driver */
	int simulation_mode;    /* 1 = enable simulation, 0 = disable */
	int verbose;            /* 1 = enable verbose logging, 0 = disable */
} scanner_config_t;

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

#endif /* RF_SCANNER_H */