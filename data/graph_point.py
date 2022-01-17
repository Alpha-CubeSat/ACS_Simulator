import matplotlib.pyplot as plt
import csv

time = []
av_angle = []


with open('pointing500delay.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')

    for row in plots:
        time.append(float(row[0]))
        av_angle.append(float(row[1]))

plt.plot(time, av_angle, label="Angle")

plt.xlabel('Time (ms)')
plt.title('IMU Update Speed 500 ms')
plt.legend()
plt.show()
