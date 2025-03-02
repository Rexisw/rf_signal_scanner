#!/usr/bin/env python3
"""
Convert IQ data from RF Signal Scanner to WAV file for easy listening.
Usage: ./convert_to_wav.py <iq_file.bin> [center_freq_hz] [sample_rate_hz]
"""

import sys
import os
import numpy as np
from scipy import signal
import scipy.io.wavfile as wavfile

def iq_to_audio(filename, center_freq=None, sample_rate=None):
    """
    Convert IQ data to audio WAV file
    If center_freq and sample_rate are not provided, they are extracted from the filename
    """
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found")
        return 1
        
    try:
        # Extract parameters from filename if not provided
        if center_freq is None or sample_rate is None:
            basename = os.path.basename(filename)
            parts = basename.split('_')
            
            # Expected format: signal_TIMESTAMP_FREQMHz_SRksps.bin
            if len(parts) >= 4:
                if center_freq is None and "MHz" in parts[2]:
                    freq_part = parts[2].split('MHz')[0]
                    try:
                        center_freq = float(freq_part) * 1e6
                        print(f"Extracted center frequency: {center_freq/1e6:.3f} MHz")
                    except ValueError:
                        print("Could not extract frequency from filename")
                        return 1
                        
                if sample_rate is None and "ksps" in parts[3]:
                    sr_part = parts[3].split('ksps')[0]
                    try:
                        sample_rate = float(sr_part) * 1000
                        print(f"Extracted sample rate: {sample_rate/1000:.1f} ksps")
                    except ValueError:
                        print("Could not extract sample rate from filename")
                        return 1
        
        # Ensure we have the required parameters
        if center_freq is None or sample_rate is None:
            print("Error: Could not determine center frequency or sample rate")
            print("Please provide them as arguments")
            return 1
            
        # Read IQ data from the binary file
        data = np.fromfile(filename, dtype=np.uint8)
        
        # Convert to complex samples (I and Q interleaved)
        iq_data = np.empty(len(data)//2, dtype=np.complex64)
        iq_data.real = (data[::2] - 127.5) / 127.5
        iq_data.imag = (data[1::2] - 127.5) / 127.5
        
        # For AM demodulation
        am_demod = np.abs(iq_data)
        
        # For FM demodulation
        # Calculate instantaneous phase
        instantaneous_phase = np.unwrap(np.angle(iq_data))
        # Calculate frequency deviation (derivative of phase)
        fm_demod = np.diff(instantaneous_phase)
        # Append a zero to maintain array size
        fm_demod = np.append(fm_demod, 0)
        
        # Apply low-pass filter for audio frequencies (15 kHz for FM broadcast)
        audio_sample_rate = 48000  # Standard audio sample rate
        
        # Decimation factor
        decim = int(sample_rate / audio_sample_rate)
        if decim < 1:
            decim = 1
            
        # Filter design
        nyq = 0.5 * sample_rate
        cutoff = 15000.0  # 15 kHz cutoff for FM audio
        b, a = signal.butter(6, cutoff/nyq, btype='low')
        
        # Filter and decimate AM
        am_audio = signal.filtfilt(b, a, am_demod)
        am_audio = am_audio[::decim]
        
        # Filter and decimate FM
        fm_audio = signal.filtfilt(b, a, fm_demod)
        fm_audio = fm_audio[::decim]
        
        # Normalize to 16-bit range
        def normalize_audio(audio):
            # Remove DC offset
            audio = audio - np.mean(audio)
            # Normalize to -1 to 1 range
            if np.max(np.abs(audio)) > 0:
                audio = audio / np.max(np.abs(audio))
            # Convert to 16-bit PCM
            return (audio * 32767).astype(np.int16)
            
        am_audio_norm = normalize_audio(am_audio)
        fm_audio_norm = normalize_audio(fm_audio)
        
        # Save WAV files
        am_out = os.path.splitext(filename)[0] + '_am.wav'
        fm_out = os.path.splitext(filename)[0] + '_fm.wav'
        
        # Calculate actual output sample rate
        output_rate = int(sample_rate / decim)
        print(f"Output sample rate: {output_rate} Hz")
        
        wavfile.write(am_out, output_rate, am_audio_norm)
        print(f"AM demodulated audio saved to: {am_out}")
        
        wavfile.write(fm_out, output_rate, fm_audio_norm)
        print(f"FM demodulated audio saved to: {fm_out}")
        
        return 0
        
    except Exception as e:
        print(f"Error processing file: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    if len(sys.argv) < 2 or len(sys.argv) > 4:
        print(f"Usage: {sys.argv[0]} <iq_file.bin> [center_freq_hz] [sample_rate_hz]")
        sys.exit(1)
        
    filename = sys.argv[1]
    
    center_freq = None
    sample_rate = None
    
    if len(sys.argv) >= 3:
        center_freq = float(sys.argv[2])
    if len(sys.argv) >= 4:
        sample_rate = float(sys.argv[3])
        
    sys.exit(iq_to_audio(filename, center_freq, sample_rate))