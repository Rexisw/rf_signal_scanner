// Code suggested by Grok
// gcc -o rf_scanner2 rf_scanner2.c -lrtlsdr -lfftw3 -lm
// ./rf_scanner2 --start 400000000 --stop 450000000 --rate 1024000 --gain 400 --time 10 --iterations 10

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <rtl-sdr.h>
#include <fftw3.h>

#define DEFAULT_SAMPLE_RATE    2048000
#define DEFAULT_FREQ_START     88000000  // 88 MHz
#define DEFAULT_FREQ_STOP      108000000 // 108 MHz
#define DEFAULT_GAIN           0         // Auto gain
#define DEFAULT_SCAN_TIME      10        // 10 seconds
#define THRESHOLD              -40       // Signal detection threshold in dB

struct scanner_config {
    uint32_t freq_start;
    uint32_t freq_stop;
    uint32_t sample_rate;
    int gain;
    int scan_time;
    int save_signal;
    int iterations;  // -1 for infinite
};

static rtlsdr_dev_t *dev = NULL;
static volatile int do_exit = 0;

void init_device(struct scanner_config *config) {
    int dev_index = 0;
    int r;

    r = rtlsdr_open(&dev, dev_index);
    if (r < 0) {
        fprintf(stderr, "Failed to open RTL-SDR device\n");
        exit(1);
    }

    rtlsdr_set_sample_rate(dev, config->sample_rate);
    rtlsdr_set_tuner_gain_mode(dev, 1);  // Manual gain
    rtlsdr_set_tuner_gain(dev, config->gain);
    rtlsdr_reset_buffer(dev);
}

uint32_t get_random_freq(uint32_t start, uint32_t stop, uint32_t step) {
    uint32_t range = (stop - start) / step;
    uint32_t random_step = rand() % range;
    return start + (random_step * step);
}

void save_signal_data(unsigned char *buf, int len, uint32_t freq, uint32_t sample_rate) {
    char filename[64];
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    
    snprintf(filename, sizeof(filename), 
            "signal_%04d%02d%02d_%02d%02d%02d_%uHz_%usps.raw",
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec,
            freq, sample_rate);

    FILE *fp = fopen(filename, "wb");
    if (fp) {
        fwrite(buf, 1, len, fp);
        fclose(fp);
        printf("Saved signal data to %s\n", filename);
    }
}

void save_spectrum_data(fftw_complex *out, int n, uint32_t freq, uint32_t sample_rate) {
    static int instance = 0;
    char filename[32];
    snprintf(filename, sizeof(filename), "spectrum_%d.csv", instance++);

    FILE *fp = fopen(filename, "w");
    if (fp) {
        fprintf(fp, "Frequency,Magnitude\n");
        double freq_step = (double)sample_rate / n;
        for (int i = 0; i < n/2; i++) {
            double mag = 20 * log10(sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]));
            double f = freq - (sample_rate/2) + (i * freq_step);
            fprintf(fp, "%.0f,%.2f\n", f, mag);
        }
        fclose(fp);
        printf("Saved spectrum data to %s\n", filename);
    }
}

int scan_frequency(struct scanner_config *config, uint32_t freq) {
    int n_read;
    unsigned char *buffer;
    fftw_complex *in, *out;
    fftw_plan plan;
    int buffer_size = config->sample_rate * config->scan_time;

    buffer = malloc(buffer_size);
    in = fftw_alloc_complex(buffer_size);
    out = fftw_alloc_complex(buffer_size);
    plan = fftw_plan_dft_1d(buffer_size, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    //rtlsdr_set_center_freq(dev, freq);
    int tmp = rtlsdr_set_center_freq(dev, freq);
    if (tmp < 0) {
        fprintf(stderr, "Failed to set frequency to %u Hz\n", freq);
        return 0;
    }
    rtlsdr_reset_buffer(dev);
    
    int r = rtlsdr_read_sync(dev, buffer, buffer_size, &n_read);
    if (r < 0) {
        fprintf(stderr, "Failed to read samples\n");
        return 0;
    }

    // Convert to complex numbers and normalize
    for (int i = 0; i < n_read; i++) {
        in[i][0] = (buffer[i] - 127.5) / 127.5;  // I component
        in[i][1] = 0;                            // Q component (simplified)
    }

    fftw_execute(plan);

    // Check for signal presence
    double max_power = -100;
    for (int i = 0; i < n_read/2; i++) {
        double power = 20 * log10(sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]));
        if (power > max_power) max_power = power;
    }

    int signal_detected = (max_power > THRESHOLD);
    if (signal_detected) {
        printf("Signal detected at %u Hz (%.1f dB)\n", freq, max_power);
        if (config->save_signal) {
            save_signal_data(buffer, n_read, freq, config->sample_rate);
        }
        save_spectrum_data(out, n_read, freq, config->sample_rate);
    }

    free(buffer);
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);

    return signal_detected;
}

int main(int argc, char *argv[]) {
    struct scanner_config config = {
        .freq_start = DEFAULT_FREQ_START,
        .freq_stop = DEFAULT_FREQ_STOP,
        .sample_rate = DEFAULT_SAMPLE_RATE,
        .gain = DEFAULT_GAIN,
        .scan_time = DEFAULT_SCAN_TIME,
        .save_signal = 1,
        .iterations = -1
    };

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--start") == 0) config.freq_start = atoi(argv[++i]);
        else if (strcmp(argv[i], "--stop") == 0) config.freq_stop = atoi(argv[++i]);
        else if (strcmp(argv[i], "--rate") == 0) config.sample_rate = atoi(argv[++i]);
        else if (strcmp(argv[i], "--gain") == 0) config.gain = atoi(argv[++i]);
        else if (strcmp(argv[i], "--time") == 0) config.scan_time = atoi(argv[++i]);
        else if (strcmp(argv[i], "--nosave") == 0) config.save_signal = 0;
        else if (strcmp(argv[i], "--iterations") == 0) config.iterations = atoi(argv[++i]);
    }

    srand(time(NULL));
    init_device(&config);
    
    int count = 0;
    while (!do_exit && (config.iterations == -1 || count < config.iterations)) {
        uint32_t freq = get_random_freq(config.freq_start, config.freq_stop, 
                                      config.sample_rate/4);
        printf("Scanning frequency: %u Hz\n", freq);
        scan_frequency(&config, freq);
        count++;
    }

    rtlsdr_close(dev);
    return 0;
}

