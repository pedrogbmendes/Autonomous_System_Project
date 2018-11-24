
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from matplotlib import colors

np.set_printoptions(threshold=np.inf) #see all matrix

img = Image.open("map5.pgm")
area = (900, 950, 1420, 1180) #left, top, right, bottom
cropped_img = img.crop(area)
img_matrix = np.array(cropped_img)
cropped_img.show()
#print img_matrix.shape

BW_img = img_matrix < 100
BW_img = BW_img * 1  #0 and 1 instead of False and True
#print BW_img

cmap = colors.ListedColormap(['Blue','red'])
plt.pcolor(BW_img[::-1], cmap=cmap, edgecolors='k')
#plt.imshow(BW_img, cmap=cmap)
plt.show()

#import numpy as np

#fl = open("map5.pgm", "r")

#assert fl.readline() == 'P5\n'
#info = fl.readline()
#(width, height) = [int(i) for i in fl.readline().split()]
#depth = int(fl.readline())
#assert depth <= 255
#print info
#print width
#print height
#print depth

#raster = []
#for y in range(height):
#    row = []
#    for y in range(width):
#        if fl.read(1)) != 205:
#            print ord(fl.read(1))
#        row.append(ord(fl.read(1)))
#
#    raster.append(row)


#photo =  np.zeros((height, width))
#for i in range(height):
#    for j in range(width):
#        photo[i][j] = ord(fl.read(1))
#
#print photo
#
#fl.close()
