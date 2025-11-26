import matplotlib.pyplot as plt
import numpy as np


gpu_pipeline_timings:np.ndarray = np.fromfile("../data/8_cpu_pipeline_timestamps.bin", dtype=np.float64)
cpu_pipeline_timings:np.ndarray = np.fromfile("../data/cpu_pipeline_timestamps.bin", dtype=np.float64)

# Calculate averages
cpu_avg = np.mean(cpu_pipeline_timings)
gpu_avg = np.mean(gpu_pipeline_timings)

#box and whisker plot
fig, axs = plt.subplots(3,1, figsize=(10,12))

axs[0].boxplot([gpu_pipeline_timings, cpu_pipeline_timings],
    label=["gpu icp","cpu_icp"])
axs[0].set_title("box and whisker plot of odometry pipeline timings")
axs[0].set_xlabel("pipeline")
axs[0].set_ylabel("time")

cpu_hist_count, cpu_hist_bins = np.histogram(cpu_pipeline_timings, bins=256)
axs[1].stairs(cpu_hist_count, cpu_hist_bins)
axs[1].axvline(cpu_avg, color='r', linestyle='--', linewidth=2, label=f'16 coreCPU Avg: {cpu_avg:.4f}')
axs[1].annotate(f'Avg: {cpu_avg:.4f}', xy=(cpu_avg, np.max(cpu_hist_count)), 
                xytext=(cpu_avg, np.max(cpu_hist_count) * 0.9),
                arrowprops=dict(arrowstyle='->', color='r'),
                fontsize=10, ha='center', color='r')
axs[1].set_title("CPU Pipeline Timings Histogram")
axs[1].set_xlabel("time")
axs[1].set_ylabel("count")
axs[1].legend()

gpu_hist_count, gpu_hist_bins = np.histogram(gpu_pipeline_timings, bins=256)
axs[2].stairs(gpu_hist_count, gpu_hist_bins)
axs[2].axvline(gpu_avg, color='r', linestyle='--', linewidth=2, label=f'8 core Avg: {cpu_avg:.4f}')
axs[2].annotate(f'Avg: {gpu_avg:.4f}', xy=(gpu_avg, np.max(gpu_hist_count)), 
                xytext=(gpu_avg, np.max(gpu_hist_count) * 0.9),
                arrowprops=dict(arrowstyle='->', color='r'),
                fontsize=10, ha='center', color='r')
axs[2].set_title("CPU Pipeline Timings Histogram")
axs[2].set_xlabel("time")
axs[2].set_ylabel("count")
axs[2].legend()

#plt.tight_layout()
plt.show()

