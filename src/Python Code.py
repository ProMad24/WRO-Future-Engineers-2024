import cv2
import numpy as np
import serial
import time
x=0
y=0
h=50
w=640
Data='F'
c='a'
try:
    ser = serial.Serial('/dev/ttyACM1', 9600,timeout=0)
except:
    ser = serial.Serial('/dev/ttyACM0', 9600,timeout=0)
area_r=0
area_g=0
red = (0, 0, 255)
green = (0, 255, 0)

cap = cv2.VideoCapture(0)
t='d'
ser.write(b'D')

#Data = ser.readline()

#Data=Data.decode('utf-8').strip()

while True:
    p=ser.read()
    p=p.decode('utf-8').strip()
    if (p=='b'):
        c='b'
    if (c=='b'):
    
        if(p=='e'):
            x=0
            y=0
            h=50
            w=640
        if(p=='n'):
            x=0
            y=0
            h=640
            w=640
        #nd=time.monotonic()
        #print(Data)
        #Data='F'
        ret, frame = cap.read()
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        

        lower_red = np.array([131, 132, 0])
        upper_red = np.array([179, 192, 255])
        lower_green = np.array([47,76,33])#28 101 0
        upper_green = np.array([82,255,255])#154 237 58

        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            R_Contour=max(contours_red,key=cv2.contourArea)
            area_r=cv2.contourArea(R_Contour)


        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_green:
            G_Contour=max(contours_green,key=cv2.contourArea)
            area_g=cv2.contourArea(G_Contour)
        if area_r > 1700 and (t !='R'or c=='a'):
            x = 250
            y = 0
            h = 640
            w= 640
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), -1)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ser.write(b'R')
          #cv2.drawContours(frame, contours_red, -1, red, 3)
          #print(p)
            t='R'
            print('r')
          
        elif area_g > 1700 and (t !='G'or c=='a'):
            x = 0
            y = 0
            h = 640
            w= 340
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), -1)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ser.write(b'G')
          #cv2.drawContours(frame, contours_green, -1, green, 3)
            t='G'
            print('g')
        elif (area_g < 1700 and area_r < 1700 ) and (t!='W' or c=='a') :
          ser.write(b'W')
          t='W'
          print('w')
        
        cv2.imshow('frame', frame)
        #cv2.imshow('re', mask_red)
        #cv2.imshow('gr', mask_green)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()