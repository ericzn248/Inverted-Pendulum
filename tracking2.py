import matplotlib.image as mpimg
import matplotlib.pyplot as plt

import cv2
import numpy as np
from numpy.linalg import inv
import time

BOUNDS = [10,60,60]
RED = [5,200,165]
BLUE = [90, 33, 122]
GREEN = [75,125,185]
PURPLE = [137,81,174]

def getMask(color):
    lower = np.array([max(0,a-b) for a,b in zip(color,BOUNDS)])
    upper = np.array([min(255,a+b) for a,b in zip(color,BOUNDS)])
    mask = cv2.inRange(HSV, lower, upper)
    return mask

def getTransM(color=BLUE):
    mask = getMask(color)
    # cv2.imshow('mask',mask)
    points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=50,param2=.30,minRadius=20,maxRadius=70)
    #points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=200,param2=0.60,minRadius=10,maxRadius=100)
    points = np.squeeze(points)
    ind = np.lexsort((points[:,1],points[:,0]))  
    points = points[ind]

    # removes accuracy value that cv2.HoughCircles returns
    points = points[:,:2]

    # sorts points into [topleft, topright, bottomleft, bottomright]
    if points[0,1] > points[1,1]: 
        temp = np.copy(points[0])
        points[0],points[1] = points[1],temp
    if points[2,1] > points[3,1]: 
        temp = np.copy(points[2])
        points[2],points[3] = points[3],temp

    points = np.array(points.tolist())
    coords = np.array([[0,30],[0,0],[100,30],[100,0]])
    trans = cv2.getPerspectiveTransform(np.float32(points),np.float32(coords))
   
    return trans

def getPoint(color=RED):
    global points
    mask = getMask(color)

    points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=50,param2=.30,minRadius=20,maxRadius=70)
    #points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=300,param2=0.750,minRadius=5,maxRadius=50)

    try:
      points = points[:,:,:2][0].reshape(-1,1,2).astype(np.float32)
      coords = cv2.perspectiveTransform(points,trans)
      coords = coords[0,:,:]
      coords = list(coords.squeeze())
    except:
      #print(color)
      return -1
    return coords

hist = []
def getVelocities(a,b,t,dt=.15):
    global hist

    
    
    return (a-b)/t

def getAngle(p1,p2):
    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]
    
    slope = dy/dx
    angle = np.arctan2((dx),(-dy))
    
    return angle * 180 / np.pi

def formatNum(x):
    return (str(x)+'00000')[:5]

def getInputs(prev_vals,dt):
    
    p1,p2,angle = prev_vals
    p1_t = getPoint(RED)
    #p1_t, p2_t = getPoint(RED), getPoint(GREEN)
    
    x,x_vel = p1_t[0], (p1_t[0]-p1[0])/dt
    toprint = ' '.join([formatNum(i) for i in [x,1/dt]]) #removed xvel
    return toprint, (x,x_vel,0,0), (p1,0,0)
    
    p1,p2 = p1_t,p2_t

    angle_t = getAngle(p1,p2)
    angle_vel = (angle_t - angle)/dt
    angle = angle_t

    toprint = ' '.join([formatNum(i) for i in [x,x_vel,angle,angle_vel,1/dt]])

    # text_display, (model_inputs), (inputs to be used as prev_vals)
    return toprint, (x,x_vel,angle,angle_vel), (p1,p2,angle)

while __name__ == "__main__":

  print('starting')

  vid = cv2.VideoCapture(1)
  vid.set(cv2.CAP_PROP_EXPOSURE, 10) 

  font = cv2.FONT_HERSHEY_SIMPLEX
  org = (50,50)
  fscale = 1
  thickness = 2

  p1,p2 = np.array((-999,-999)), np.array((-999,-999))
  t = 0
  x_vel=0
  angle=0

  toprint = ''
  prev_t = time.time()
  t = 0
  xx = 0
  prev = ((-1,-1),(-1,-1),-1)
  t = time.time()

  while True:
      ret,img = vid.read()
      img = cv2.resize(img,(1600,900))
      prev_t = t
      t = time.time()
      if ret == False: break
      blur = cv2.GaussianBlur(img, (11,11), cv2.BORDER_DEFAULT)
      HSV = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
      
      if xx%50==0:
        try:
          trans = getTransM()
        except Exception as e:
          print(e)

      try:
        toprint, vals, prev = getInputs(prev,t-prev_t)
      except Exception as e:
        pass
      # img = cv2.resize(img,(1000,562))
      img = cv2.putText(img,toprint,org,font,fscale,(0,0,0),thickness)
      cv2.imshow('red',getMask(BLUE))
      cv2.imshow('img',img)
      cv2.waitKey(1)
      xx += 1



  vid.release()
  cv2.destroyAllWindows()


