#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include "../include/rf_scanner.h"

/* Test random frequency generation */
void test_random_frequency() {
    double start = 88e6;
    double stop = 108e6;
    uint32_t sample_rate = 2.4e6;
    
    for (int i = 0; i < 100; i++) {
        double freq = random_frequency(start, stop, sample_rate);
        assert(freq >= start && freq <= stop);
        
        /* Check that frequency step is reasonable based on sample rate */
        double step = sample_rate / 4.0;
        double remainder = fmod(freq - start, step);
        assert(remainder < 1.0);  /* Allow for small floating point errors */
    }
    
    printf("Random frequency test passed\n");
}

/* Test argument parsing */
void test_argument_parsing() {
    scanner_config_t config;
    char *argv[] = {"rf_scanner", "-f", "100000000", "-F", "200000000", 
                   "-s", "1000000", "-g", "20", "-t", "5", "-S", "0", "-i", "10"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    
    int result = parse_arguments(argc, argv, &config);
    assert(result == 0);
    
    assert(config.start_freq == 100000000.0);
    assert(config.stop_freq == 200000000.0);
    assert(config.sample_rate == 1000000);
    assert(config.gain == 20.0);
    assert(config.scan_time == 5);
    assert(config.save_signal == 0);
    assert(config.iterations == 10);
    
    free(config.fft_output_file);
    printf("Argument parsing test passed\n");
}

int main() {
    printf("Running scanner tests...\n");
    
    test_random_frequency();
    test_argument_parsing();
    
    printf("All tests passed!\n");
    return 0;
}