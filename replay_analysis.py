from matplotlib import pyplot as plt
import numpy as np
import argparse

# ================== CONFIG ========================
times_us_path = "/home/axby/times_us.txt";
commanded_qdots_path = "/home/axby/commanded_qdots.txt";
actual_qdots_path = "/home/axby/actual_qdots.txt";
# ===================================================

parser = argparse.ArgumentParser("Replay Analysis")
parser.add_argument("--separate-plots", action="store_true", default=False)
args = parser.parse_args()

with open(times_us_path, "r") as f:
    lines = f.readlines()
    times_us = np.array([int(num.strip()) for num in lines])

with open(commanded_qdots_path, "r") as f:
    lines = f.readlines()
    commanded_qdots = np.array([[float(num.strip()) for num in line.split()] for line in lines])

with open(actual_qdots_path, "r") as f:
    lines = f.readlines()
    actual_qdots = np.array([[float(num.strip()) for num in line.split()] for line in lines])

ts = times_us/1e6

if args.separate_plots:
    plt.figure(figsize=(12, 8))
    for i in range(7):
        plt.subplot(4, 2, i+1)  # 4x2 grid to accommodate 7 joints
        plt.plot(ts, commanded_qdots[:,i], label=f'Joint {i} Desired', linestyle='--')
        plt.plot(ts, actual_qdots[:,i], label=f'Joint {i} Actual')
        plt.xlabel('Time (s)')
        plt.ylabel('q̇')
        plt.legend()
        plt.grid(True)
    plt.tight_layout()
    plt.show()
else:
    colors = ['red', 'green', 'blue', 'maroon', 'purple', 'gray', 'orange']

    # qdots
    legend = []
    for i in range(7):
        plt.plot(ts, commanded_qdots[:,i], linewidth=0.7, c=colors[i],
                 linestyle='--')
        legend.append(f"commanded qdot{i}")

    for i in range(7):
        plt.plot(ts, actual_qdots[:,i], linewidth=0.7, c=colors[i],
                 linestyle='-')
        legend.append(f"actual qdot{i}")
    plt.legend(legend)    
    plt.show()
