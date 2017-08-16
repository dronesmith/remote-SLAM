import cv2
import imutils
import numpy as np;
import time
import sys
from matplotlib import pyplot as plt

import sys
import getopt


print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)

print sys.argv[1]
camera = cv2.VideoCapture(sys.argv[1]) #cv2.VideoCapture(0)



(grabbed, im) = camera.read()
height, width, chan = np.shape(im)
print( height, width/2)

# Define the codec and create VideoWriter object
fourcc = cv2.cv.FOURCC(*'mp4v')

outL = cv2.VideoWriter('outputL.mp4',fourcc, 20.0, (width/2,height))
outR = cv2.VideoWriter('outputR.mp4',fourcc, 20.0, (width/2,height))

imL0= im[:height, :width/2 ]
imR0= im[:height, width/2:width]
cv2.imwrite('LeftCam/Lpic0.png',imL0)
cv2.imwrite('RightCam/Rpic0.png',imR0)
i=0
while (camera.isOpened()):
    #(grabbed, im) = camera.read()
    (grabbed, im) = camera.read()

    imL= im[:height, :width/2 ]
    cv2.imwrite('LeftCam/Lpic%d.png' %i,imL)
    cv2.imshow('imL',imL)
    print 'imL Size: ', np.shape(imL)
    #imL=cv2.resize(imL, (height, width/2))
    outL.write(imL)


    imR = im[:height, width/2:width]
    cv2.imshow('imR',imR)
    cv2.imwrite('RightCam/Rpic%d.png' %i,imR)
    print 'imR Size: ', np.shape(imL)
    #imR=cv2.resize(imR, (height, width/2))
    outR.write(imR)


    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    i=i+1
camera.release()
outL.release()
outR.release()
cv2.destroyAllWindows()
