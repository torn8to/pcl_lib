import matplotlib.pyplot as plt
import numpy as np


cpu8_pipeline_timings: np.ndarray = np.fromfile("../data/8_cpu_pipeline_timestamps.bin", dtype=np.float64)
cpu16_pipeline_timings: np.ndarray = np.fromfile("../data/16_cpu_pipeline_timestamps.bin", dtype=np.float64)

# Calculate averages
cpu_avg = np.mean(cpu16_pipeline_timings)
cpu8_avg = np.mean(cpu8_pipeline_timings)

# box and whisker plot + combined histogram
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

# Box plot
axs[0].boxplot(
    [cpu8_pipeline_timings, cpu16_pipeline_timings],
    tick_labels=["8 core icp", "16 core icp"]
)
axs[0].set_title("Box and Whisker Plot of Odometry Pipeline Timings")
axs[0].set_xlabel("Pipeline")
axs[0].set_ylabel("Time")
axs[0].axhline(
    y=0.1,
    color="grey",
    linestyle="--",
    linewidth=1.5,
    label="0.1 s 10hz lidar rate" 
)
axs[0].legend(loc="upper right")

# Combined histogram

bins = 128
cpu_hist_count, cpu_hist_bins = np.histogram(cpu16_pipeline_timings, bins=bins)
cpu8_hist_count, cpu8_hist_bins = np.histogram(cpu8_pipeline_timings, bins=bins)

cpu_plot = axs[1].stairs(cpu_hist_count, cpu_hist_bins, color='tab:blue', label="16 core Histogram")
gpu_plot = axs[1].stairs(cpu8_hist_count, cpu8_hist_bins, color='tab:orange', label="8 core Histogram")

cpu_line = axs[1].axvline(cpu_avg, color='tab:blue', linestyle='--', linewidth=2, label=f'16 core Avg: {cpu_avg:.4f}')
gpu_line = axs[1].axvline(cpu8_avg, color='tab:orange', linestyle='--', linewidth=2, label=f'8 core Avg: {cpu8_avg:.4f}')

axs[1].annotate(f'CPU Avg: {cpu_avg:.4f}',
                xy=(cpu_avg, np.max(cpu_hist_count)),
                xytext=(cpu_avg, np.max(cpu_hist_count) * 0.9),
                arrowprops=dict(arrowstyle='->', color='tab:blue'),
                fontsize=10,
                ha='center',
                color='tab:blue')

axs[1].annotate(f'8 core Avg: {cpu8_avg:.4f}',
                xy=(cpu8_avg, np.max(cpu8_hist_count)),
                xytext=(cpu8_avg, np.max(cpu8_hist_count) * 0.9),
                arrowprops=dict(arrowstyle='->', color='tab:orange'),
                fontsize=10,
                ha='center',
                color='tab:orange')

axs[1].set_title("Pipeline Timings Histogram")
axs[1].set_xlabel("Time")
axs[1].set_ylabel("Count")
axs[1].legend()

plt.tight_layout()
plt.show()

