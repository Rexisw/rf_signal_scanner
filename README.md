# RF Signal Scanner

A simple C implementation of a scanner using an RTL SDR or similar device.

## Overview

The scanner operates in the following way:
* Randomly picks a frequency within a predefined frequency range. The frequency steps are determined based on the sampling rate.
* Scans the selected frequency with the predefined sample rate for a predefined time, e.g., 10s.
* If a signal is detected, the recorded signal is saved to a file. The filename will contain a timestamp, frequency and sample rate.
* Creates an FFT of the sample data and stores spectrum data in a CSV file. A new unique file is created for each application instance.
* Repeats the process for the specified number of iterations or until interrupted.

## Requirements

- librtlsdr (RTL-SDR library)
- fftw3 (Fast Fourier Transform library)
- gcc (GNU Compiler Collection)
- make (build tool)

On Debian/Ubuntu systems, you can install the dependencies with:
```bash
sudo apt-get install librtlsdr-dev libfftw3-dev gcc make
```

## Building

To build the project:
```bash
make
```

This will create the `rf_scanner` executable in the project root directory.

## Usage

```
./rf_scanner [options]

Options:
  -f <start_freq>  Start frequency in Hz (default: 88000000 Hz)
  -F <stop_freq>   Stop frequency in Hz (default: 108000000 Hz)
  -s <sample_rate> Sample rate in Hz (default: 2400000 Hz)
  -g <gain>        Gain in dB (default: 40.0 dB)
  -t <scan_time>   Time to scan each frequency in seconds (default: 10 s)
  -S <0|1>         Save signal data to file (default: 1)
  -i <iterations>  Number of iterations (0 = infinite, default: 0)
  -h               Show this help message
```

### Examples

Scan the FM broadcast band (88-108 MHz) with default settings:
```bash
./rf_scanner
```

Scan the 2-meter ham band (144-148 MHz) with a 1 MHz sample rate for 5 seconds per frequency:
```bash
./rf_scanner -f 144000000 -F 148000000 -s 1000000 -t 5
```

Scan the WiFi 2.4 GHz band for 100 iterations with signal saving disabled:
```bash
./rf_scanner -f 2400000000 -F 2500000000 -s 2000000 -S 0 -i 100
```

## Output Files

The scanner generates two types of output files:

1. Signal files: Binary IQ data files containing the raw samples from detected signals
   - Format: `signal_YYYYMMDD_HHMMSS_XXX.XXXMHZ_XXXXksps.bin`

2. Spectrum files: CSV files containing frequency and power data from the FFT analysis
   - Format: `spectrum_YYYYMMDD_HHMMSS_XXX.XXXMHZ_XXXXksps.csv`

## Testing

To run the tests:
```bash
cd tests
make
./test_scanner
```

## License

This project is open source software.

