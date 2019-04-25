import matplotlib.pyplot as plt
import csv
import numpy as np

def main():
    data1 = []  #roll angle
    data2 = []  #pitch angle
    data3 = []  #yaw angle
    data4 = []  #distance between the arms
    data5 = []  #workspace shared volume


    with open('/home/varun/sai2/apps/dualarm_mobile_sim/01-kinematic_sim/bimanual_data_v1.csv','r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        for row in plots:
            data1.append(float(row[0]))
            data2.append(float(row[1]))
            data3.append(float(row[2]))
            data4.append(float(row[3]))
            data5.append(float(row[4]))


    plt.figure(0)
    plt.plot(data3,data5)
    plt.xlabel('Yaw Angle [rad]')
    plt.ylabel('Shared Workspace Volume [m3]')
    plt.title('Change in Shared Workspace Volume with Yaw Angle')
    plt.grid()
    plt.show()





if __name__ == "__main__":
  main()



