import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
file_path = 'speed_preprocessoracc_speed_limit.csv'
data = pd.read_csv(file_path, header=None)

# 提取时间和限速列
time = data[0]
steer_angle = data[1]
yaw_rate = data[2]
fliter_steer_angle = data[3]
fliter_yaw_rate = data[4]
steering_angle_change_rate = data[5]
yaw_rate_change_rate = data[6]
weigh = data[7]
current_kappa = data[8]
path_kappa = data[9]
path_speed_limit = data[10]
speed_limit = data[11]

# 提取曲率列（倒数第二列）
curvature = data[data.shape[1] - 2]

# 计算转弯半径（避免除以0，设置一个很小的值作为阈值）
curvature[curvature == 0] = 1e-6  # 避免除以0
turning_radius = 1 / curvature

# 创建图形窗口
fig, ax1 = plt.subplots(figsize=(10, 6))

# 绘制限速随时间的变化图
ax1.plot(time, path_kappa, label='path_kappa', color='blue')
ax1.plot(time, current_kappa, label='current_kappa', color='red', linestyle='--')
#ax1.plot(time, yaw_rate, label='yaw_rate', color='pink', linestyle='--')

ax1.set_xlabel('Time')
ax1.set_ylabel('value', color='blue')
ax1.tick_params(axis='y', labelcolor='blue')
ax1.legend(loc='upper right')


# 添加标题
#plt.title('Speed Limit and Turning Radius Over Time')

# 显示网格
plt.grid(True)

# 显示图形
plt.show()
