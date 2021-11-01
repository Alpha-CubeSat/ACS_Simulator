import matplotlib.pyplot as plt
import csv

time = []
av_p = []
av_q = []
av_r = []

with open('500delay.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')

    for row in plots:
        time.append(float(row[0]))
        av_p.append(float(row[1]))
        av_q.append(float(row[2]))
        av_r.append(float(row[3]))

plt.plot(time, av_p, label="P Angular Velocity")
plt.plot(time, av_q, label="Q Angular Velocity")
plt.plot(time, av_r, label="R Angular Velocity")

plt.xlabel('Time (ms)')
plt.title('IMU Update Speed 500 ms')
plt.legend()
plt.show()
