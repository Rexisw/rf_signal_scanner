#!/usr/bin/env python3
"""
Plot spectrum data from RF Signal Scanner.
Usage: ./plot_spectrum.py <spectrum_file.csv>
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_spectrum(filename):
    """Plot spectrum data from a CSV file"""
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        return 1
        
    try:
        # Read the CSV file
        data = pd.read_csv(filename)
        
        # Extract frequency and power columns
        freq = data['Frequency']
        power = data['Power']
        
        # Calculate center frequency and bandwidth
        center_freq = (freq.max() + freq.min()) / 2
        bandwidth = freq.max() - freq.min()
        
        # Create the plot
        plt.figure(figsize=(12, 6))
        plt.plot(freq / 1e6, power, 'b-')
        plt.grid(True)
        plt.xlabel('Frequency (MHz)')
        plt.ylabel('Power (dBFS)')
        plt.title(f'RF Spectrum - Center: {center_freq/1e6:.3f} MHz, BW: {bandwidth/1e6:.3f} MHz')
        
        # Improving the plot with additional details
        plt.fill_between(freq / 1e6, power, -100, alpha=0.3)
        
        # Add vertical line at max power point
        max_power_idx = np.argmax(power)
        max_power_freq = freq[max_power_idx] / 1e6
        max_power = power[max_power_idx]
        plt.axvline(x=max_power_freq, color='r', linestyle='--', alpha=0.7)
        plt.text(max_power_freq + 0.1, max_power, 
                 f'{max_power_freq:.3f} MHz\n{max_power:.1f} dBFS', 
                 fontsize=9, bbox=dict(facecolor='white', alpha=0.7))
        
        # Extract timestamp and frequency from filename
        basename = os.path.basename(filename)
        parts = basename.split('_')
        if len(parts) >= 3:
            timestamp = parts[1]
            plt.figtext(0.02, 0.02, f'Timestamp: {timestamp}', fontsize=8)
            
        # Adjust plot limits
        plt.xlim(freq.min() / 1e6, freq.max() / 1e6)
        plt.ylim(min(power.min(), -80), max(power.max() + 5, -20))
        
        # Save the plot
        output_file = os.path.splitext(filename)[0] + '.png'
        plt.savefig(output_file, dpi=150)
        print(f"Plot saved to {output_file}")
        
        # Show the plot
        plt.show()
        
        return 0
        
    except Exception as e:
        print(f"Error processing file: {e}")
        return 1

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <spectrum_file.csv>")
        sys.exit(1)
    
    sys.exit(plot_spectrum(sys.argv[1]))