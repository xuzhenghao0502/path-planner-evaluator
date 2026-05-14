
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

# 读取CSV文件并删除空值
data = pd.read_csv('/home/zhinengzhang@geometricalpal.com/eka/app/pnc/pnc_motion_planner/speed_planner/src/speed_optimizerrisk_speed_limit.csv', sep=r'\s+', header=None)
data.dropna(inplace=True)

# 获取空间点和时间点的数量
s_values = np.arange(0, 150)  # 空间点
t_values = [0.0, 0.1, 0.2, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.2, 3.6, 4.0, 4.4, 4.8, 5.0]   # 时间点

#------------------------------------------------------------------------------------------
# 创建图形和轴
fig, ax = plt.subplots()
X, Y = np.meshgrid(s_values, t_values)

# 设置轴标签
ax.set_xlabel('Space Point')
ax.set_ylabel('Time Point')

for frame in range(data.shape[0]):
# 初始化等高图，这里我们使用第一行数据作为初始数据
    v_values = data.iloc[frame, :].values.reshape(16, 150)
    ax.cla()  # 清除当前轴的内容
    contour = ax.contourf(Y, X, v_values, cmap='viridis',levels=150)

# 添加颜色条
    # cbar = plt.colorbar(contour, ax=ax)
    # 显示图形
    plt.title(f'Frame: {frame}')
    plt.draw()  # 绘制当前帧
    plt.pause(0.1)  # 暂停一会儿，以便观察
    
    # 清除当前图形，为下一帧做准备
    # plt.clf()



# # 更新函数，用于动画
# def update(frame):
#     # global v_values
#     # 更新v_values为当前帧的数据
#     v_values = data.iloc[frame, :].values.reshape(16, 150)
#     contour.set_array(v_values.ravel())  # 更新等高图数据
#     return contour,

# # 创建动画
# ani = animation.FuncAnimation(fig, update, frames=data.shape[0], blit=False)



#----------------------------------------           
# 创建图形和3D轴
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # 创建一个网格，其中每个点的坐标是(s, t)
# X, Y = np.meshgrid(s_values, t_values)

# # 初始化三维图，这里我们使用特定行的数据作为示例
# # 请确保data是一个pandas DataFrame，并且包含了足够的数据
# v_values = data.iloc[100, :].values.reshape(16, 150)  # 选择DataFrame中特定行的数据

# # 绘制三维曲面图
# surf = ax.plot_surface(X, Y, v_values, cmap='viridis')

# # 添加颜色条
# cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)  # 调整颜色条的大小和比例

# # 设置轴标签
# ax.set_xlabel('Space Point')
# ax.set_ylabel('Time Point')
# ax.set_zlabel('Value')  # 设置Z轴标签，这里假设Z轴表示v_values的值





# 显示图形
plt.show()