# Standard imports
import cv2
import imutils
import numpy as np;
import time
from matplotlib import pyplot as plt


# Read image
#im = cv2.imread("images/photo1.jpg", cv2.IMREAD_GRAYSCALE)
camera = cv2.VideoCapture(0)
(grabbed, im) = camera.read()
height, width, chan = np.shape(im)
print( width, height)

#Fonts
font = cv2.FONT_HERSHEY_SIMPLEX


# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10;
params.maxThreshold = 200;

# Filter by Area.
params.filterByArea = False
params.minArea = 1500

# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.1

# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.87

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.01

# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
    detector = cv2.SimpleBlobDetector(params)
else :
    detector = cv2.SimpleBlobDetector_create(params)


# Set up the detector with default parameters.
detector = cv2.SimpleBlobDetector()
firstFrame = None
FS = False
def nothing(*arg):
    pass

cv2.namedWindow('Trackbars')
cv2.createTrackbar('Threshold', 'Trackbars', 0, 255, nothing )
cv2.createTrackbar('H_Max', 'Trackbars', 0, 255, nothing )
cv2.createTrackbar('S_Min', 'Trackbars', 0, 255, nothing )
cv2.createTrackbar('S_Max', 'Trackbars', 0, 255, nothing )
cv2.createTrackbar('V_Min', 'Trackbars', 0, 255, nothing )
cv2.createTrackbar('V_Min', 'Trackbars', 0, 255, nothing )




while True:
    #Trackbar reads
    thrs1 = cv2.getTrackbarPos('Threshold','Trackbars')
    hue_max = cv2.getTrackbarPos('H_Max','Trackbars')
    sat_min = cv2.getTrackbarPos('S_Min','Trackbars')
    sat_max = cv2.getTrackbarPos('S_Max','Trackbars')
    val_min = cv2.getTrackbarPos('V_Min','Trackbars')
    val_max = cv2.getTrackbarPos('V_Max','Trackbars')


    #Grab image frame and resize
    (grabbed, im) = camera.read()
    if FS == False:
        (grabbed, im) = camera.read()
        imL= im[:height, :width/2 ]
        imR = im[:height, width/2:width]
    elif FS == True:
        (grabbed, imL) = camera.read()
        time.sleep(10/1000.0)
        (grabbed, imR) = camera.read()
    #im = imutils.resize(im, width=500)

    #Gray image and apply GaussianBlur
    grayL = cv2.cvtColor(imL, cv2.COLOR_BGR2GRAY)
    grayL = cv2.GaussianBlur(grayL, (15, 15), 0)

    grayR = cv2.cvtColor(imR, cv2.COLOR_BGR2GRAY)
    grayR = cv2.GaussianBlur(grayR, (15, 15), 0)

    #initial detection

    #Calc threshold image

    frameDelta = cv2.absdiff(grayL, grayR)
    thresh = cv2.threshold(frameDelta, thrs1, 255, cv2.THRESH_BINARY)[1]

    #thresh = cv2.dilate(thresh, None, iterations=2)
    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Detect blobs.
    keypointsL = detector.detect(grayL)
    keypointsR = detector.detect(grayR)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypointsR = cv2.drawKeypoints(grayR, keypointsL, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    im_with_keypointsL = cv2.drawKeypoints(grayL, keypointsR, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    #Stero Map
    stereo = cv2.StereoBM(2,16,15)
    disparity = stereo.compute(grayR,grayL)
    normalizedImg = np.zeros((height, width/2))

    disparity2= cv2.normalize(disparity, disparity, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U);
    #plt.imshow(disparity,'gray')
    #plt.show()
    cv2.imshow("Disparity", disparity2)
    cv2.moveWindow("Disparity", width, 0)

    # Show keypoints

    cv2.imshow("Keypoints R", im_with_keypointsR)
    cv2.imshow("Keypoints L", im_with_keypointsL)
    cv2.moveWindow("Keypoints L", 0, 0)
    cv2.moveWindow("Keypoints R", width/2, 0)


    cv2.imshow("Grey and blurred L", grayL)
    cv2.imshow("Grey and blurred R", grayR)
    cv2.moveWindow("Grey and blurred L", 0, height)
    cv2.moveWindow("Grey and blurred R", width/2, height)


    #cv2.imshow("Frame Delta", frameDelta)
    #cv2.moveWindow("Frame Delta", width, 0)

    #cv2.putText(frame, command,(10,50), font, .8,(255,255,255),2,cv2.cv.CV_AA)
    cv2.putText(thresh,"%i"%thrs1, (10,50), font, .8,(100,100,100),1,cv2.cv.CV_AA)
    cv2.imshow("Threshold", thresh)
    cv2.moveWindow("Threshold", width, height)

    cv2.imshow("imL", imL)
    cv2.moveWindow("imL", 0, 2*height)
    cv2.imshow("imR", imR)
    cv2.moveWindow("imR", width/2, 2*height)
    cv2.waitKey(1)
