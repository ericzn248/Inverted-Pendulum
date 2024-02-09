import matplotlib.image as mpimg
import matplotlib.pyplot as plt

import cv2
import numpy as np
from numpy.linalg import inv
import time

CALPOINTS = []

# hsv_value = cv2.cvtColor(img[y,x].reshape(1,1,-1), cv2.COLOR_BGR2HSV)[0,0].tolist()

#add keep max and min to adjust color
class color(object):
    RGB = [0, 0, 0]
    MIN = [255, 255, 255]
    MAX = [0, 0, 0]
    N = 0
    done = False
    name = ""
    def __init__(self, name): #  color, bounds = [10, 10, 10]
        a = 1
        self.name = name
        #self.RGB = color
        # self.MIN = [max(0, a-b) for a,b in zip(color, bounds)]
        # self.MAX = [min(255, a+b) for a,b in zip(color, bounds)]

    def update(self, r, g, b):
        self.N += 1
        #update averages
        self.RGB[0] += r
        self.RGB[1] += g
        self.RGB[2] += b
        delta = 5 if self.name == 'blue' else 25
        #update bounds
        self.MIN[0] = min(self.MIN[0], r - delta)
        self.MIN[1] = min(self.MIN[1], g - delta)
        self.MIN[2] = min(self.MIN[2], b - delta)
        self.MAX[0] = max(self.MAX[0], r + delta)
        self.MAX[1] = max(self.MAX[1], g + delta)
        self.MAX[2] = max(self.MAX[2], b + delta)
        return 1
    
    def getRGB(self):
        return [self.RGB[0]/self.N, self.RGB[1]/self.N, self.RGB[2]/self.N]
    
    def inRange(self, j):
        lower = np.array([max(0, b) for a, b in zip(self.RGB, self.MIN)])
        upper = np.array([min(255, b) for a, b in zip(self.RGB, self.MAX)])
        if (lower[0] < j[0] < upper[0] and lower[1] < j[1] < upper[1] and lower[2] < j[2] < upper[2]):
            return True
        return False

    def getMask(self, mat):
        lower = np.array([max(0, b) for a, b in zip(self.RGB, self.MIN)])[::-1]
        upper = np.array([min(255, b) for a, b in zip(self.RGB, self.MAX)])[::-1]
        mask = cv2.inRange(mat, lower, upper)
        return mask

def getBlur():
    ret, img = vid.read()

    img = cv2.resize(img, (800, 450))
    # prev_t = t
    # t = time.time()
    if ret == False:
        return -1
    blur = cv2.GaussianBlur(img, (11, 11), cv2.BORDER_DEFAULT)
    return blur

def getTransM(color):
    blur = getBlur()
    mask = color.getMask(blur)
    # cv2.imshow('mask',mask)
    points = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT_ALT,
        1,
        100,
        param1=50,
        param2=0.30,
        minRadius=10,
        maxRadius=100,
    )

    print(points[0], len(points))
    if len(points[0]) == 4:
        #print("setting calpoitns")
        global CALPOINTS
        CALPOINTS = points[0]

    # points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=200,param2=0.60,minRadius=10,maxRadius=100)
    points = np.squeeze(points)
    ind = np.lexsort((points[:, 1], points[:, 0]))
    points = points[ind]

    # removes accuracy value that cv2.HoughCircles returns
    points = points[:, :2]

    # sorts points into [topleft, topright, bottomleft, bottomright]
    if points[0, 1] > points[1, 1]:
        temp = np.copy(points[0])
        points[0], points[1] = points[1], temp
    if points[2, 1] > points[3, 1]:
        temp = np.copy(points[2])
        points[2], points[3] = points[3], temp

    points = np.array(points.tolist())
    coords = np.array([[0, 30], [0, 0], [100, 30], [100, 0]])
    trans = cv2.getPerspectiveTransform(np.float32(points), np.float32(coords))

    return trans

def getPoint(color):
    global points

    blur = getBlur()
    mask = color.getMask(blur)

    points = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT_ALT,
        1,
        100,
        param1=50,
        param2=0.30,
        minRadius=10,
        maxRadius=100,
    )
    # points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=300,param2=0.750,minRadius=5,maxRadius=50)
    # if (color.name != "red"):
    #     print(points)
    try:
        points = points[:, :, :2][0].reshape(-1, 1, 2).astype(np.float32)
        coords = cv2.perspectiveTransform(points, trans)
        coords = coords[0, :, :]
        coords = list(coords.squeeze())
    except:
        # print(color)
        return -1
    return coords

hist = []

def getVelocities(a, b, t, dt=0.15):
    global hist
    return (a - b) / t

def getAngle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    slope = dy / dx
    angle = np.arctan2((dx), (-dy))

    return angle * 180 / np.pi

def formatNum(x):
    return (str(x) + "00000")[:5]

def getInputs(prev_vals, dt):
    p1, p2, angle = prev_vals
    p1_t = getPoint(red)

    # print(p1_t, red.MIN, red.MAX)
    # p1_t, p2_t = getPoint(RED), getPoint(GREEN)

    x, x_vel = p1_t[0], (p1_t[0] - p1[0]) / dt
    toprint = " ".join([formatNum(i) for i in [x, 1 / dt]])  # removed xvel
    return toprint, (x, x_vel, 0, 0), (p1, 0, 0)

    p1, p2 = p1_t, p2_t
    angle_t = getAngle(p1, p2)
    angle_vel = (angle_t - angle) / dt
    angle = angle_t
    toprint = " ".join([formatNum(i) for i in [x, x_vel, angle, angle_vel, 1 / dt]])
    # text_display, (model_inputs), (inputs to be used as prev_vals)
    return toprint, (x, x_vel, angle, angle_vel), (p1, p2, angle)

def click_data_blue(event, x, y, flags, param):
    img = param
    if (event == cv2.EVENT_LBUTTONDOWN):
        # print(x,' , ', y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y,x,0]
        g = img[y,x,1]
        r = img[y,x,2]
        text = str(r) + ',' + str(g) + ',' + str(b) #RGB comes here
        if (r == 0 and b == 0 and g == 0):
            blue.done = True
            return
        
        print(text)
        print(blue.MIN, blue.MAX)
        blue.update(r, g, b)
        print(blue.MIN, blue.MAX)
        print()

def click_data_red(event, x, y, flags, param):
    img = param
    if (event == cv2.EVENT_LBUTTONDOWN):
        # print(x,' , ', y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y,x,0]
        g = img[y,x,1]
        r = img[y,x,2]
        text = str(r) + ',' + str(g) + ',' + str(b) #RGB comes here
        if (r == 0 and b == 0 and g == 0):
            red.done = True
            return
        red.update(r, g, b)

def runSetup():
    while True:
        ret, img = vid.read()
        img = cv2.resize(img, (800, 450))
        blur = cv2.GaussianBlur(img, (11, 11), cv2.BORDER_DEFAULT)
        # HSV = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        cv2.imshow("blue", blue.getMask(blur)) #

        img  = cv2.rectangle(img, (750, 400), (800, 450), (0, 0, 0), -1) 
        img = cv2.putText(img, "choose blue calibration circles", org, font, fscale, (0, 0, 0), thickness)
        
        cv2.imshow("img", img)
        cv2.setMouseCallback('img',click_data_blue,img)       
        cv2.waitKey(1)

        if (blue.done):
            break
    cv2.destroyAllWindows()

    time.sleep(0.5)
    red.MIN = [255, 255, 255]
    red.MAX = [0, 0, 0]
    print(blue.MIN, blue.MAX, red.MIN, red.MAX)

    while True:
        ret, img = vid.read()
        img = cv2.resize(img, (800, 450))
        cv2.imshow("red", red.getMask(blur)) #
        img  = cv2.rectangle(img, (750, 400), (800, 450), (0, 0, 0), -1) 
        img = cv2.putText(img, "choose red calibration circles", org, font, fscale, (0, 0, 0), thickness)
        cv2.imshow("img", img)
        cv2.setMouseCallback('img',click_data_red,img)       
        cv2.waitKey(1)

        if (red.done):
            break
    cv2.destroyAllWindows()

    print(blue.MIN, blue.MAX, red.MIN, red.MAX)
#exit()
blue = color("blue")
red = color("red")
print(blue.name, red.name)

vid = cv2.VideoCapture(1)
vid.set(cv2.CAP_PROP_EXPOSURE, 10)

font = cv2.FONT_HERSHEY_SIMPLEX
org = (50, 50)
fscale = 1
thickness = 2

p1, p2 = np.array((-999, -999)), np.array((-999, -999))
t = 0
x_vel = 0
angle = 0
# trans = None
# blur = None

def setup():
    toprint = ""
    prev_t = time.time()
    t = 0
    xx = 0
    prev = ((-1, -1), (-1, -1), -1)
    t = time.time()

    global trans,blur
    iteration = 0
    runSetup()

    while True:
        iteration += 1
        ret, img = vid.read()

        img = cv2.resize(img, (800, 450))
        prev_t = t
        t = time.time()
        if ret == False:
            break
        blur = cv2.GaussianBlur(img, (11, 11), cv2.BORDER_DEFAULT)
        # HSV = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        if xx % 50 == 0:
            try:
                trans = getTransM(blue)
            except Exception as e:
                print("Calibration circles error: ", e)
        try:
            toprint, vals, prev = getInputs(prev, t - prev_t)
        except Exception as e:
            pass
        img = cv2.putText(img, toprint, org, font, fscale, (0, 0, 0), thickness)
        
        for p in CALPOINTS:
            img = cv2.circle(img, (int(p[0]), int(p[1])), int(p[2]), (255, 0, 0), 2)

        cv2.imshow("img", img)
        cv2.waitKey(1)
        xx += 1
        if xx == 100: break

    # vid.release()
    cv2.destroyAllWindows()