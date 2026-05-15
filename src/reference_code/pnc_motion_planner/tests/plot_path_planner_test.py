import math
import os
import csv
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import matplotlib.ticker as ticker
import textwrap

lane_change_dirs = ['lane_change_data.csv']
lane_keep_dirs = ['lane_keep_data.csv']
current_dirs = lane_change_dirs + lane_keep_dirs

for current_dir in current_dirs:
    try:
        # 读取 CSV 文件
        with open(current_dir, "r", encoding="utf-8") as file:
            reader = csv.reader(file)
            info = [next(file).rstrip() for _ in range(6)]
            coordinates1 = [row for row in reader]

        # 将坐标转换为 NumPy 数组
        coordinates1 = np.array(coordinates1, dtype=float)

        # 创建网格布局并设置图像大小
        fig = plt.figure(figsize=(16, 9))  # 可以根据需要调整图像大小
        gs = gridspec.GridSpec(3, 2, width_ratios=[1, 1], wspace=0.4, hspace=0.3)
        axs1= plt.subplot(gs[:3, 0])
        axs2= plt.subplot(gs[0, 1])
        axs3= plt.subplot(gs[1, 1])
        axs4= plt.subplot(gs[2, 1])

        # 绘制所有子图
        axs1.plot(coordinates1[:, 2], coordinates1[:, 1], "-b", linewidth=1.5, label="reference_line")
        axs1.plot(coordinates1[:, 6], coordinates1[:, 5], "-g", linewidth=1.5, label="path")
        axs1.plot(coordinates1[:, 13], coordinates1[:, 12], "-r", linewidth=1.0, label="barrier_bound_right")
        axs1.plot(coordinates1[:, 15], coordinates1[:, 14], "-r", linewidth=1.0, label="barrier_bound_left")
        axs1.plot(coordinates1[:, 17], coordinates1[:, 16], "-y", linewidth=0.5, label="soft_bound_right")
        axs1.plot(coordinates1[:, 19], coordinates1[:, 18], "-y", linewidth=0.5, label="soft_bound_left")
        axs1.set_title("PathPlanResult")
        axs1.set_xlabel("Y-axis")
        axs1.set_ylabel("X-axis")
        axs1.invert_xaxis()
        axs1.axis("equal")
        axs1.legend()

        axs2.plot(coordinates1[:, 0], coordinates1[:, 10], "-r", label="kappa")
        axs2.set_xlabel("X-axis")
        axs2.set_ylabel("Y-axis")
        axs2.legend()

        axs3.plot(coordinates1[:, 0], coordinates1[:, 11], "-b", label="dkappa")
        axs3.set_xlabel("X-axis")
        axs3.set_ylabel("Y-axis")
        axs3.legend()


        for j, line in enumerate(info):
            # 使用textwrap.wrap函数来自动换行，设定每行的最大字符数
            wrapped_text = textwrap.wrap(line, width=90)
            # 将换行后的文本连接成一个字符串，每行之间用\n分隔
            formatted_text = '\n'.join(wrapped_text)
            # 在图表中添加自动换行的文本
            axs4.text(0.0, 1.0 - 0.1 * j, formatted_text, color='red', fontsize=10, ha='left', va='top', wrap=True)
        axs4.axis('off')

        # 使用紧凑布局
        # plt.tight_layout()
        plt.savefig(current_dir.replace('.csv', '.png'), dpi=300, bbox_inches='tight')
        plt.close()  # 关闭图形以释放内存

    except FileNotFoundError:
        print(f"File not found: {current_dir}")
        continue

# 显示最后一个图形（如果需要）
plt.show()
