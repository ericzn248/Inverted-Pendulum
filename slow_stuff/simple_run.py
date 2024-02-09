import comms
import jtracking as t
import time
import tkinter as tk
import math
import cv2
import keyboard



# vid = cv2.VideoCapture(1)
# ret,img = vid.read()
# # img = cv2.resize(img,(800,450))
# blur = cv2.GaussianBlur(img, (11,11), cv2.BORDER_DEFAULT)
# HSV = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)


stopped = True
leftRight = True # right = true


def checkBoundary(xavg):
    global stopped
    if xavg < 25 or xavg> 75:
        comms.stop()
        return True
    return False
    
def reset():
    global stopped
    cv2.destroyAllWindows()
    middle = 50
    stopped = True
    cartLoc = t.getPoint(t.red)
    if(cartLoc==-1):
        print("system fail")
        return
    while(abs(cartLoc[0]-middle)>4):
        print(cartLoc[0])
        if cartLoc[0] < middle: 
            comms.moveRight()
        elif cartLoc[0]>middle: 
            comms.moveLeft()
        cartLoc = t.getPoint(t.red)
        cancancan.update()

def startstop():
    global stopped
    stopped = not stopped
    cv2.destroyAllWindows()
    print(stopped)

def toggle():
    global leftRight
    leftRight = not leftRight

def rocks(x1, y1, angle, r, frame=-1, reward=-1):
    newcan = tk.Toplevel(cancancan)
    newcan.title("Display")
    myCanvas = tk.Canvas(newcan,bg='white', height=500, width=1000)
    myCanvas.pack()
    x2 = x1+r*math.cos(angle)
    y2  = y1+r*math.sin(angle)
    #print(x2, y2)
    pendulum = myCanvas.create_line(x1,y1,x1+r*math.cos(angle),y1+r*math.sin(angle),fill='red')
    myCanvas.create_text(250,50,fill="darkblue",font="Times 12 italic",text=f"{frame}  {reward}")
    # myCanvas.coords(pendulum,500,250,500+200*math.cos(angle),250+200*math.sin(angle))
    myCanvas.update()

time.sleep(3) #Wait five seconds to ensure that its set up



cancancan = tk.Tk()
cancancan.title("Homescreen")
# cancancan.geometry("1100x600")
# window = tk.Label(text="I love rocks",width=50,height=25)

background = tk.PhotoImage(file='pendulum.png')
bglabel = tk.Label(cancancan,image=background)
# bglabel.place(x=0,y=0)
bglabel.pack()

frame = tk.Frame(cancancan)
frame.place(x=550,y=300)
# frame.pack(pady=20)
startBtn = tk.Button(frame,text="Start/Stop",command=startstop)
resetBtn = tk.Button(frame,text="Reset",command=reset)
toggleBtn = tk.Button(frame,text="Toggle Left/Right",command=toggle)
startBtn.pack(pady=20)
resetBtn.pack()
toggleBtn.pack()

# startBtn.bind("<Button-1>",startstop)
# resetBtn.bind("<Button-1>",reset)

# cancancan.mainloop()
cancancan.update()

lastTime = time.time()

try:
    #comms.moveRight()
    print("starting")
    t.setup()

    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50,50)
    fscale = 1
    thickness = 2
    toprint = ''
    ct = 0
    
    # for i in range(50):
    #     comms.moveRight()
    #     angle = comms.getAngle()
    #     myCanvas.create_line(500,250,500-100*math.cos(angle+3*math.pi/2),250-100*math.sin(angle+3*math.pi/2),fill='red')
    #     myCanvas.update()
    #     myCanvas.delete('all')
    #     time.sleep(0.01)
    #     comms.moveLeft()
    #     angle = comms.getAngle()
    #     time.sleep(0.01)
    #     myCanvas.create_line(500,250,500-100*math.cos(angle+3*math.pi/2),250-100*math.sin(angle+3*math.pi/2),fill='red')
    #     myCanvas.update()
    #     myCanvas.delete('all')

    while True:
        # stopped = False
        if not stopped:

            ret,img = t.vid.read()
            if ret == False: break

            if leftRight:
                for i in range(1): comms.moveRight()
                # for i in range(2): comms.moveLeft()
            else:
                # for i in range(2): comms.moveRight()
                for i in range(1): comms.moveLeft()
            
            # pastCartX = [50,50,50]
            # cartLoc = t.getPoint(HSV,BOUNDS,RED)
            # pastCartX[0] = pastCartX[1]
            # pastCartX[1] = pastCartX[2]
            # pastCartX[2] = cartLoc

            # stopped = checkBoundary(sum(pastCartX)/3)
            cartLoc = t.getPoint(t.red)
            stopped = checkBoundary(cartLoc[0])

            if ct%20==0:
                try:
                    trans = t.getTransM(t.blue)
                except Exception as e:
                    print(e)
            try:
                toprint = t.getInputs(1,1)
            except Exception as e:
                pass
            img = cv2.putText(img,toprint,org,font,fscale,(0,0,0),thickness)

            # img = cv2.resize(img,(800,450))
            # cv2.imshow('img',img)
            cv2.waitKey(1)
            ct += 1
            print(f"FPS: {1/(time.time()-lastTime)}")
            lastTime = time.time()
        # cancancan.update_idletasks()
        cancancan.update()


except KeyboardInterrupt:
    comms.stop()