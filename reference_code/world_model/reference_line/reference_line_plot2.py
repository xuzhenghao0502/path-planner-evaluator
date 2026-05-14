import math
import os
import csv
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import matplotlib.ticker as ticker

#current_dir = current_dir = os.path.dirname(os.path.abspath(__file__)) + "/srcmemorized_route_sliced_0.csv"
raw_dirs = ['srcmemorized_route_sliced_{}.csv'.format(m) for m in range(0, 206)]
left_dirs = ['srcmemorized_route_sliced_{}_left.csv'.format(m) for m in range(0, 206)]
right_dirs = ['srcmemorized_route_sliced_{}_right.csv'.format(m) for m in range(0, 206)]
current_dirs = raw_dirs + left_dirs + right_dirs

for current_dir in current_dirs:
    try:
        # 读取 CSV 文件
        with open(current_dir, "r", encoding="utf-8") as file:
            reader = csv.reader(file)
            info = [next(file).rstrip() for _ in range(11)]
            coordinates1 = [row for row in reader]

        # 将坐标转换为 NumPy 数组
        coordinates1 = np.array(coordinates1, dtype=float)

        # 创建网格布局并设置图像大小
        fig = plt.figure(figsize=(16, 9))  # 可以根据需要调整图像大小
        gs = gridspec.GridSpec(3, 2, width_ratios=[1, 1], wspace=0.4, hspace=0.3)
        axs1= plt.subplot(gs[:2, 0])
        axs2= plt.subplot(gs[0, 1])
        axs3= plt.subplot(gs[1, 1])
        axs4= plt.subplot(gs[2, 0])
        axs5= plt.subplot(gs[2, 1])

        # 绘制所有子图
        axs1.plot(coordinates1[:, 1], coordinates1[:, 2], "-r", label="raw")
        axs1.plot(coordinates1[:, 3], coordinates1[:, 4], "-b", label="smooth")
        axs1.set_title("ReferenceLine")
        axs1.set_xlabel("X-axis")
        axs1.set_ylabel("Y-axis")
        axs1.axis("equal")
        axs1.legend()

        axs2.plot(coordinates1[:, 0], coordinates1[:, 5], "-r", label="kappa")
        axs2.set_xlabel("X-axis")
        axs2.set_ylabel("Y-axis")
        axs2.legend()

        axs3.plot(coordinates1[:, 0], coordinates1[:, 6], "-b", label="dkappa")
        axs3.set_xlabel("X-axis")
        axs3.set_ylabel("Y-axis")
        axs3.legend()

        axs4.plot(coordinates1[:, 0], coordinates1[:, 7], "-y", label="offset")
        axs4.set_xlabel("X-axis")
        axs4.set_ylabel("Y-axis")
        axs4.legend()

        for j, line in enumerate(info):
            axs5.text(0.0, 1.0 - 0.1 * j, line, color='red', fontsize=10, ha='left', va='top', wrap=True)
        axs5.axis('off')

        # 使用紧凑布局
        # plt.tight_layout()
        plt.savefig(current_dir.replace('.csv', '.png'), dpi=300, bbox_inches='tight')
        plt.close()  # 关闭图形以释放内存

    except FileNotFoundError:
        print(f"File not found: {current_dir}")
        continue

# 显示最后一个图形（如果需要）
plt.show()
