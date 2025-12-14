#!/usr/bin/env python3
import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def analyze_csv(path):
    df = pd.read_csv(path)
    if 't' not in df.columns:
        raise ValueError('CSV doit contenir colonne t')
    times = df['t'].to_numpy()
    joints = [c for c in df.columns if c != 't']
    duration = times[-1] - times[0]
    stats = {'duration': duration, 'joints': {}}
    for j in joints:
        vals = df[j].to_numpy()
        amp_min = vals.min()
        amp_max = vals.max()
        # vitesse moyenne (abs diff / dt) moyenne sur l'intervalle
        dt = np.diff(times)
        dv = np.abs(np.diff(vals))
        speeds = dv / (dt + 1e-12)
        mean_speed = speeds.mean() if len(speeds)>0 else 0.0
        stats['joints'][j] = {'min': float(amp_min), 'max': float(amp_max), 'mean_speed': float(mean_speed)}

    # plot
    plt.figure(figsize=(10, 6))
    for j in joints:
        plt.plot(times - times[0], df[j], label=j)
    plt.xlabel('time (s)')
    plt.ylabel('position')
    plt.title(os.path.basename(path))
    plt.legend()
    out_png = path + '.png'
    plt.savefig(out_png)
    plt.close()
    return stats, out_png

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: analyze.py <trajectory.csv>')
        sys.exit(1)
    path = sys.argv[1]
    stats, out_png = analyze_csv(path)
    print('Stats:')
    print(f"  Duration: {stats['duration']:.3f} s")
    for j, v in stats['joints'].items():
        print(f"  Joint {j}: min={v['min']:.4f}, max={v['max']:.4f}, mean_speed={v['mean_speed']:.4f}")
    print(f'Plot saved to: {out_png}')
