import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import cv2
import numpy as np
from numpy.linalg import inv
import time

def getMask(color):
    lower = np.array([a-b for a,b in zip(color,bounds)])
    upper = np.array([a+b for a,b in zip(color,bounds)])
    mask = cv2.inRange(hsv, lower, upper)
    return mask

def getTransM(color=[106, 120, 157]):
    mask = getMask(BLUE)
    
    points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=200,param2=0.60,minRadius=10,maxRadius=100)
    points = np.squeeze(points)
    ind = np.lexsort((points[:,1],points[:,0]))  
    points = points[ind]
    if points[0,1] > points[1,1]: 
        temp = np.copy(points[0])
        points[0],points[1] = points[1],temp
    if points[2,1] > points[3,1]: 
        temp = np.copy(points[2])
        points[2],points[3] = points[3],temp
    points = points[:,:2]

    points = np.array(points.tolist())
    coords = np.array([[0,0],[0,50],[100,0],[100,50]]) #TL BL TR BR
    trans = cv2.getPerspectiveTransform(np.float32(points),np.float32(coords))
   
    return trans

def getPoint(color=[4,150,152]):
    global points
    mask = getMask(color)
    
    points = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT_ALT,1,100,param1=300,param2=0.750,minRadius=5,maxRadius=70)

    try:
      points = points[:,:,:2][0].reshape(-1,1,2).astype(np.float32)
      coords = cv2.perspectiveTransform(points,trans)
      coords = coords[0,:,:]
      coords = list(coords.squeeze())
    except:
      return -1
    return coords #x, y

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

def getInputs():

    p1 = getPoint(GREEN)

    # angle_t = getAngle(p1,p2)
    # angle_vel = (angle_t - angle)/t
    # angle = angle_t

    toprint = f'{formatNum(p1[0])} {formatNum(p1[1])}'

    return toprint

    

print('starting')
bounds = [10,70,70]
RED = [170,100,250]
BLUE = [106, 120, 157]
GREEN = [45,117,150]

while __name__ == "__main__":
  vid = cv2.VideoCapture(0)
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
  prev = ((-1,-1),(-1,-1),-1,time.time())
  while True:
    
      ret,img = vid.read()
      img = cv2.imread("lucascam.JPG")
      if ret == False: break
      img = cv2.resize(img,(800,450))
      blur = cv2.GaussianBlur(img, (11,11), cv2.BORDER_DEFAULT)
      hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
      
      if xx%20==0:
        try:
          trans = getTransM(hsv)
        except Exception as e:
          print(e)

      try:
        toprint= getInputs()
      except Exception as e:
        pass

      img = cv2.putText(img,toprint,org,font,fscale,(0,0,0),thickness)
      img = getMask(BLUE)
      print(getPoint(BLUE))
      cv2.imshow('img',img)
      cv2.waitKey(1)
      xx += 1
  vid.release()
  cv2.destroyAllWindows()


