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

np.set_printoptions(threshold=np.inf) #see all matrix

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

plt.figure(1)
cmap = colors.ListedColormap(['grey', 'yellow', 'black'])
plt.pcolor(map[::-1], cmap=cmap, edgecolors='k')
count = 1
n = 0;


size_vector = 40
h_v = np.zeros((size_vector, 1))
middle = int(np.floor(size_vector/2))
points = np.zeros((2,size_vector))
p_xs =  np.zeros((2,size_vector))
direction_plot  = np.zeros((2,200))
x_sinit = 300
y_sinit = 30
x_s = x_sinit
y_s = y_sinit

#all the angles are between -pi(exclusive) and pi(inclusive)
#predicted orientation
new_yaw = 2*np.pi/3
if new_yaw <= -np.pi:
    new_yaw += 2*np.pi
elif new_yaw > np.pi:
    new_yaw -= 2*np.pi

if new_yaw > 0 and new_yaw <= np.pi/2 :
    incr_x = 0.83
    incr_y = 0.17
elif new_yaw > np.pi/2  and new_yaw <= np.pi:
    incr_x = 0.17
    incr_y = 0.17
elif new_yaw > -np.pi/2  and new_yaw <= 0:
    incr_x = 0.83
    incr_y = 0.83
elif new_yaw > -np.pi and new_yaw <= -np.pi/2 :
    incr_x = 0.17
    incr_y = 0.83


#determine the plane of photo (parallel to the photo taken)
direction1 = new_yaw - (np.pi/2)
if direction1 <= -np.pi:
    direction1 += 2*np.pi
elif direction1 > np.pi:
    direction1 -= 2*np.pi

direction2 = new_yaw + (np.pi/2)
if direction2 <= -np.pi:
    direction2 += 2*np.pi
elif direction2 > np.pi:
    direction2 -= 2*np.pi


if direction1 > 0 and direction1 <= np.pi/2:
    incr_xd1 = 0.83
    incr_yd1 = 0.17
elif direction1 > np.pi/2 and direction1 <= np.pi:
    incr_xd1 = 0.17
    incr_yd1 = 0.17
elif direction1 > -np.pi/2 and direction1 <= 0:
    incr_xd1 = 0.83
    incr_yd1 = 0.83
elif direction1 > -np.pi and direction1 <= -np.pi/2:
    incr_xd1 = 0.17
    incr_yd1 = 0.83

if direction2 > 0 and direction2 <= np.pi/2 :
    incr_xd2 = 0.83
    incr_yd2 = 0.17
elif direction2 > np.pi/2  and direction2 <= np.pi:
    incr_xd2 = 0.17
    incr_yd2 = 0.17
elif direction2 > -np.pi/2  and direction2 <= 0:
    incr_xd2 = 0.83
    incr_yd2 = 0.83
elif direction2 > -np.pi and direction2 <= -np.pi/2 :
    incr_xd2 = 0.17
    incr_yd2 = 0.83

flag_out = 0
count_pixels = 1
distance_max = count_pixels * resolution

for i in range (middle, size_vector):

    if map[y_s, x_s] != 0:
        x_s = x_s_i
        y_s = y_s_i
        if new_yaw > 0 and new_yaw <= np.pi:
            ang_aux = -np.pi/2;
        else:
            ang_aux = np.pi/2;

        while map[y_s, x_s] != 0 and y_s in range(0, width_map):
            y_s = int(np.floor( y_s - np.sin(ang_aux) + incr_y))


    x_m = int(np.floor( x_s + np.cos(new_yaw) + incr_x))
    y_m = int(np.floor( y_s - np.sin(new_yaw) + incr_y))

    if flag_out == 1:
        flag_out = 0
        x_s = x_s_i
        y_s = y_s_i

    #prediction of position point by the camera
    #Stops when find a obstacle or reachs the max range of camera (5 meters)
    while map[y_m, x_m] != 1 and distance_max < 5 and x_m in range(0, length_map) and y_m in range(0, width_map):
        x_m = int(np.floor(x_m + np.cos(new_yaw) + incr_x))
        y_m = int(np.floor(y_m - np.sin(new_yaw) + incr_y))
        count_pixels += 1
        distance_max = count_pixels * resolution
        if count == 1:
            direction_plot[:,n] = [x_m, y_m]
            n += 1

    count = 0
    points[:,i] = [x_m, y_m]
    vector_dis = np.array([[x_s-x_m],[y_s -y_m]])
    h_v[i] = LA.norm(vector_dis)

    p_xs[:,i] = [x_s, y_s]
    x_s = int(np.floor( x_s + np.cos(direction1) + incr_xd1))
    y_s = int(np.floor( y_s - np.sin(direction1) + incr_yd1))
    x_s_i = x_s
    y_s_i = y_s
    count_pixels = 1
    distance_max = count_pixels * resolution

    while map[y_s, x_s] != 0 and distance_max < 5 and x_s in range(0, length_map) and y_s in range(0, width_map):
        x_s = int(np.floor( x_s + np.cos(new_yaw) + incr_x))
        y_s = int(np.floor( y_s - np.sin(new_yaw) + incr_y))
        count_pixels += 1
        distance_max = count_pixels * resolution
    if map[y_s, x_s] == 0 and distance_max != resolution:
        flag_out = 1

x_s = int(np.floor( x_sinit + np.cos(direction2) + incr_xd2))
y_s = int(np.floor( y_sinit - np.sin(direction2) + incr_yd2))

flag_out = 0
count_pixels = 1
distance_max = count_pixels * resolution

for j in range (middle-1, -1, -1):

    if map[y_s, x_s] != 0:
        x_s = x_s_i
        y_s = y_s_i
        if new_yaw > 0 and new_yaw <= np.pi:
            ang_aux = -np.pi/2;
        else:
            ang_aux = np.pi/2;

        while map[y_s, x_s] != 0 and y_s in range(0, width_map):
            y_s = int(np.floor( y_s - np.sin(ang_aux) + incr_y))

    x_m = int(np.floor( x_s + np.cos(new_yaw) + incr_x))
    y_m = int(np.floor( y_s - np.sin(new_yaw) + incr_y))

    if flag_out == 1:
        flag_out = 0
        x_s = x_s_i
        y_s = y_s_i

    #prediction of position point by the camera
    #Stops when find a obstacle or reachs the max range of camera (5 meters)
    while map[y_m, x_m] != 1 and distance_max < 5 and x_m in range(0, length_map) and y_m in range(0, width_map):
        x_m =  int(np.floor(x_m + np.cos(new_yaw) + incr_x))
        y_m =  int(np.floor(y_m - np.sin(new_yaw) + incr_y))
        count_pixels += 1
        distance_max = count_pixels * resolution

    points[:,j] = [x_m, y_m];
    vector_dis = np.array([[x_s-x_m],[y_s-y_m]])
    h_v[j] = LA.norm(vector_dis)
    p_xs[:,j] = [x_s, y_s]

    x_s = int(np.floor( x_s + np.cos(direction2) + incr_xd2))
    y_s = int(np.floor( y_s - np.sin(direction2) + incr_yd2))
    x_s_i = x_s
    y_s_i = y_s
    count_pixels = 1
    distance_max = count_pixels * resolution

    while map[y_s, x_s] != 0 and distance_max < 5 and x_s in range(0, length_map) and y_s in range(0, width_map):
        x_s = int(np.floor( x_s + np.cos(new_yaw) + incr_x))
        y_s = int(np.floor( y_s - np.sin(new_yaw) + incr_y))
        count_pixels += 1
        distance_max = count_pixels * resolution
    if map[y_s, x_s] == 0 and distance_max != resolution:
        flag_out = 1





y_plot = 179-points[1,:]
x_plot = points[0,:]
x_dir_plot  = direction_plot[0,0:n]
y_dir_plot  = 179 -direction_plot[1,0:n]
x_s_p = x_sinit
y_s_p = 179-y_sinit
direction_plot
plt.pcolor(map[::-1], cmap=cmap, edgecolors='k')
plt.plot(x_s_p, y_s_p, 'x')
plt.plot(x_plot,y_plot, '.')
ys_plot = 179-p_xs[1,:]
xs_plot = p_xs[0,:]
plt.plot(x_plot,y_plot, '.')
plt.plot(xs_plot,ys_plot, '.')
plt.plot(x_dir_plot,y_dir_plot, '.')
plt.show()
