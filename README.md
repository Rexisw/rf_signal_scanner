# rf_signal_scanner
SDR scanner

This is a simple C implementation of a scanner using an RTL SDR or similar.

The scanner will operate in the following way:
* Randomly pick a frequency within a predefined frequency range. The frequency steps are determined based on the sampling rate.
* Scan the selected frequency with the predefined sample rate for a predefined time, e.g., 10s.
* If a signal is detected, the recorded signal is saved to a file. The filename will contain a timestamp, frequency and sample rate.
* Create an FFT of the sample data and store spectrum data file. A new unique file is created each application instance. File format is csv.
* Start over

Input parameters:
* Start frequency
* Stop frequency
* Sample rate
* Gain control
* Time to scan each frequency
* Enable/disable save signal data to file
* Number of iterations or time to run

