"""
@file noise_analysis.py
@brief Computes electrical noise variance, peak-to-peak noise, and steady-state RMSE.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def analyze_steady_state_noise(csv_path: str = None):
    # Generate steady-state synthetic dataset matching report parameters if no file passed
    if csv_path is None:
        np.random.seed(101)
        time_s = np.arange(0, 60, 1.0) # 60 seconds sampling
        ref_temp = 25.0 # Constant 25.0°C room temp benchmark
        
        # Exact standard deviations and noise from empirical report
        t_ds = ref_temp + np.random.normal(0, 0.1527, size=len(time_s))
        t_ntc = ref_temp + np.random.normal(0, 0.1982, size=len(time_s))
        
        # Fused sensor weighted blend (80% DS18B20 / 20% NTC at steady state)
        t_fused = (0.80 * t_ds) + (0.20 * t_ntc)
        
        df = pd.DataFrame({'time_s': time_s, 't_ref': ref_temp, 't_ds': t_ds, 't_ntc': t_ntc, 't_fused': t_fused})
    else:
        df = pd.read_csv(csv_path)

    # Metrics computation
    metrics = {}
    for col in ['t_ds', 't_ntc', 't_fused']:
        std_dev = np.std(df[col])
        p2p = np.max(df[col]) - np.min(df[col])
        rmse = np.sqrt(np.mean((df[col] - df['t_ref'])**2))
        metrics[col] = {'std_dev': std_dev, 'p2p': p2p, 'rmse': rmse}

    print("=" * 65)
    print("             STEADY-STATE NOISE & ACCURACY ANALYSIS               ")
    print("=" * 65)
    print(f"{'Sensor Signal':<18} | {'Std Dev (σ) (°C)':<16} | {'Peak-to-Peak (°C)':<16} | {'RMSE (°C)':<10}")
    print("-" * 65)
    print(f"{'DS18B20 Digital':<18} | {metrics['t_ds']['std_dev']:<16.4f} | {metrics['t_ds']['p2p']:<16.4f} | {metrics['t_ds']['rmse']:<10.4f}")
    print(f"{'NTC Calibrated':<18} | {metrics['t_ntc']['std_dev']:<16.4f} | {metrics['t_ntc']['p2p']:<16.4f} | {metrics['t_ntc']['rmse']:<10.4f}")
    print(f"{'Fused Output':<18} | {metrics['t_fused']['std_dev']:<16.4f} | {metrics['t_fused']['p2p']:<16.4f} | {metrics['t_fused']['rmse']:<10.4f}")
    print("=" * 65)

    # Visualizing Noise Profiles
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    
    axes[0].plot(df['time_s'], df['t_ds'], color='blue', label='DS18B20 (Digital)')
    axes[0].axhline(25.0, color='black', linestyle='--')
    axes[0].set_ylabel('Temp (°C)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.4)

    axes[1].plot(df['time_s'], df['t_ntc'], color='orange', label='NTC Thermistor (Analog)')
    axes[1].axhline(25.0, color='black', linestyle='--')
    axes[1].set_ylabel('Temp (°C)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.4)

    axes[2].plot(df['time_s'], df['t_fused'], color='green', linewidth=1.8, label='Fused Signal Output')
    axes[2].axhline(25.0, color='black', linestyle='--')
    axes[2].set_xlabel('Time (Seconds)')
    axes[2].set_ylabel('Temp (°C)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.4)

    plt.suptitle('Steady-State Electrical Noise Profile Comparison')
    plt.tight_layout()
    plt.savefig('noise_analysis_plot.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    analyze_steady_state_noise()