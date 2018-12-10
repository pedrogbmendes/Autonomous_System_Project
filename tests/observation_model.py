import numpy as np
from numpy import linalg as LA
from transforms3d import quaternions
import roslib
import sys
import rospy
from sensor_msgs.msg import Image, Imu
import time
import math
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib import colors

resolution = 0.05 #meters/pixel

np.set_printoptions(threshold=4) #see all matrix

img = Image.open("map.pgm")
area = (950, 950, 1600, 1130) #left, top, right, bottom
cropped_img = img.crop(area)
img_matrix = np.array(cropped_img)


#unknown positions of map
BW_img_des = img_matrix == 205
BW_img_des = BW_img_des * - 1
#occupied positions of the map
BW_img_oc = img_matrix == 0
BW_img_oc = BW_img_oc* 1  #0 and 1 instead of False and True

map = BW_img_des+BW_img_oc
length_map = map.shape[1]#no of columns
width_map = map.shape[0]#no of rows


count = 1
n = 0;


size_vector = 360
h_v = np.zeros((size_vector, 1))
middle = int(np.floor(size_vector/2))
points = np.zeros((2,size_vector))
p_xs =  np.zeros((2,size_vector))
direction_plot  = np.zeros((2,2000))
x_sinit = 20
y_sinit = 140
x_s = x_sinit
y_s = y_sinit


#all the angles are between -pi(exclusive) and pi(inclusive)
#predicted orientation
new_yaw = np.pi/2
while new_yaw <= -np.pi:
    new_yaw += 2*np.pi
while new_yaw > np.pi:
    new_yaw -= 2*np.pi

margin_angle = np.pi/60

count_pixels = 1
distance_max = count_pixels * resolution

incr_angle = (29.0*np.pi)/(180*(size_vector/2))
angle_incre = new_yaw

x_incr = x_s
x_m = x_s
y_m = y_s

for i in range (middle, size_vector):

    #prediction of position point by the camera
    #Stops when find a obstacle or reachs the max range of camera (5 meters)
    while map[y_m, x_m] == 0 and distance_max < 5 and x_m in range(0, length_map) and y_m in range(0, width_map):
        while angle_incre <= -np.pi:
            angle_incre += 2*np.pi
        while angle_incre > np.pi:
            angle_incre -= 2*np.pi


        if angle_incre >= 0 and angle_incre <= np.pi/2:
            #first quadrant
            aux_yaw = angle_incre
        elif angle_incre > np.pi/2 and angle_incre <=np.pi:
            #sendon quadrant
            aux_yaw = np.pi - angle_incre
        elif angle_incre < 0 and angle_incre >= -np.pi/2:
            #fourth quadrant
            aux_yaw = -angle_incre
        elif angle_incre >= -np.pi and angle_incre <-np.pi/2:
            #thrid quadrant
            aux_yaw = angle_incre + np.pi


        if aux_yaw > np.pi/3 and aux_yaw < 7*np.pi/18:
            #between 60 and 70 degrees
            increment = 0.05
        elif aux_yaw >= 7*np.pi/18 and aux_yaw < 8*np.pi/18:
            #between 70 and 80 degrees
            increment = 0.01
        elif aux_yaw >= 8*np.pi/18 and aux_yaw < 85*np.pi/180:
            #between 80 an 85 degrees
            increment = 0.005
        elif aux_yaw >= 85*np.pi/180 and aux_yaw <= np.pi/2:
            #between 85 an 90 degrees
            increment = 0.001
        else:
            increment = 0.1


        if angle_incre >= 0 and angle_incre < np.pi/2-margin_angle :
            #first quadrant
            x_incr = x_incr + increment
            x_m = int(x_incr)
            y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

        elif angle_incre > np.pi/2+margin_angle  and angle_incre <= np.pi:
            #sencond quadrant
            x_incr = x_incr - increment
            x_m = int(x_incr)
            y_m = int(-np.tan(np.pi-angle_incre)*( x_s - x_incr ) + y_s)

        elif angle_incre > -np.pi/2+margin_angle  and angle_incre < 0:
            #fourth quadrant
            x_incr = x_incr + increment
            x_m = int(x_incr)
            y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

        elif angle_incre > -np.pi-margin_angle and angle_incre < -np.pi/2 :
            #third quadrant
            x_incr = x_incr - increment
            x_m = int(x_incr)
            y_m = int(-np.tan(-np.pi-angle_incre)*(x_s - x_incr) + y_s)

        elif angle_incre > np.pi/2-margin_angle and angle_incre < np.pi/2+margin_angle:
            y_m = int(y_m - 1)

        elif angle_incre > -np.pi/2-margin_angle and angle_incre < -np.pi/2+margin_angle:
            y_m = int(y_m + 1)

        count_pixels += 1
        distance_max = count_pixels * resolution * increment

        if count == 1:
            direction_plot[:,n] = [x_m, y_m]
            n += 1

    count = 0
    points[:,i] = [x_m, y_m]

    p_radial = np.array([[x_s-x_m],[y_s-y_m]])
    dis_radial = LA.norm(p_radial)
    h_v[i] = resolution *dis_radial*np.cos(angle_incre-new_yaw)
    angle_incre -= incr_angle

    count_pixels = 1
    distance_max = count_pixels * resolution
    x_m = x_s
    y_m = y_s
    x_incr = x_s


count_pixels = 1
distance_max = count_pixels * resolution
angle_incre = new_yaw + incr_angle
x_m = x_s
y_m = y_s
x_incr = x_s


for j in range (middle-1, -1, -1):

    #prediction of position point by the camera
    #Stops when find a obstacle or reachs the max range of camera (5 meters)
    while map[y_m, x_m] == 0 and distance_max < 5 and x_m in range(0, length_map) and y_m in range(0, width_map):
        while angle_incre <= -np.pi:
            angle_incre += 2*np.pi
        while angle_incre > np.pi:
            angle_incre -= 2*np.pi

        if angle_incre >= 0 and angle_incre <= np.pi/2:
            #first quadrant
            aux_yaw = angle_incre
        elif angle_incre > np.pi/2 and angle_incre <=np.pi:
            #sendon quadrant
            aux_yaw = np.pi - angle_incre
        elif angle_incre < 0 and angle_incre >= -np.pi/2:
            #fourth quadrant
            aux_yaw = -angle_incre
        elif angle_incre >= -np.pi and angle_incre <-np.pi/2:
            #thrid quadrant
            aux_yaw = angle_incre + np.pi

        if aux_yaw > np.pi/3 and aux_yaw < 7*np.pi/18:
            #between 60 and 70 degrees
            increment = 0.05
        elif aux_yaw >= 7*np.pi/18 and aux_yaw < 8*np.pi/18:
            #between 70 and 80 degrees
            increment = 0.01
        elif aux_yaw >= 8*np.pi/18 and aux_yaw < 85*np.pi/180:
            #between 80 an 85 degrees
            increment = 0.005
        elif aux_yaw >= 85*np.pi/180 and aux_yaw <= np.pi/2:
            #between 85 an 90 degrees
            increment = 0.001
        else:
            increment = 0.1

        if angle_incre >= 0 and angle_incre < np.pi/2-margin_angle :
            #first quadrant
            x_incr = x_incr + increment
            x_m = int(x_incr)
            y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

        elif angle_incre > np.pi/2+margin_angle  and angle_incre <= np.pi:
            #sencond quadrant
            x_incr = x_incr - increment
            x_m = int(x_incr)
            y_m = int(-np.tan(np.pi-angle_incre)*( x_s - x_incr ) + y_s)

        elif angle_incre > -np.pi/2+margin_angle  and angle_incre < 0:
            #fourth quadrant
            x_incr = x_incr + increment
            x_m = int(x_incr)
            y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

        elif angle_incre > -np.pi-margin_angle and angle_incre < -np.pi/2 :
            #third quadrant
            x_incr = x_incr - increment
            x_m = int(x_incr)
            y_m = int(-np.tan(-np.pi-angle_incre)*(x_s - x_incr) + y_s)

        elif angle_incre > np.pi/2-margin_angle and angle_incre < np.pi/2+margin_angle:
            y_m = int(y_m - 1)

        elif angle_incre > -np.pi/2-margin_angle and angle_incre < -np.pi/2+margin_angle:
            y_m = int(y_m + 1)


        count_pixels += 1
        distance_max = count_pixels * resolution * increment

    points[:,j] = [x_m, y_m];

    p_radial = np.array([[x_s-x_m],[y_s-y_m]])
    dis_radial = LA.norm(p_radial)
    h_v[j] = resolution *dis_radial*np.cos(angle_incre-new_yaw)
    angle_incre += incr_angle

    count_pixels = 1
    distance_max = count_pixels * resolution
    x_m = x_s
    y_m = y_s
    x_incr = x_s


y_plot = 179-points[1,:]
x_plot = points[0,:]
x_dir_plot  = direction_plot[0,0:n]
y_dir_plot  = 179 -direction_plot[1,0:n]
x_s_p = x_sinit
y_s_p = 179-y_sinit
print h_v

plt.figure(1)
cmap = colors.ListedColormap(['grey', 'yellow', 'black'])
plt.pcolor(map[::-1], cmap=cmap, edgecolors='k')
plt.plot(x_s_p, y_s_p, 'x')
plt.plot(x_plot,y_plot, '.')
plt.plot(x_dir_plot,y_dir_plot, '.')
plt.show()
