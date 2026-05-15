import math
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

current_dir = current_dir = os.path.dirname(os.path.abspath(__file__))

with open(current_dir + "/srcmemorized_route_sliced_1.csv", "r", encoding="utf-8") as file:
    reader = csv.reader(file)
    
    info = [file.readline().rstrip() for _ in range(11)]    
    for j in range(0, 11):
        next(file)

    coordinates1 = []
    for row in reader:
        coordinates1.append([float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7])])



coordinates1 = np.array(coordinates1)

for i in range(0, len(coordinates1)):
    plt.clf()
    plt.plot(coordinates1[:, 1], coordinates1[:, 2], "-r", label="raw")
    plt.plot(coordinates1[:, 3], coordinates1[:, 4], "-b", label="smooth")
    #plt.xticks(range(-20,20,2))
    #plt.yticks(range(-20,20,2))
    plt.title("ReferenceLine")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.axis("equal")
    plt.legend()

    plt.pause(0.02)

plt.show()
