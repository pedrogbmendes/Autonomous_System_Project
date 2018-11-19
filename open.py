

import numpy as np

fl = open("map6.pgm", "r")


assert fl.readline() == 'P5\n'
info = fl.readline()
(width, height) = [int(i) for i in fl.readline().split()]
depth = int(fl.readline())
assert depth <= 255
print info
print width
print height
print depth

raster = []
for y in range(height):
    row = []
    for y in range(width):
        if ord(fl.read(1)) != 205:
            print ord(fl.read(1))
        row.append(ord(fl.read(1)))

    raster.append(row)


#photo =  np.zeros((height, width))
#for i in range(height):
#    for j in range(width):
#        photo[i][j] = float(fl.read(1))

#print photo
