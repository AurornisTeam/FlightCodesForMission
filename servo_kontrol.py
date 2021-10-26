
#KIRMIZI HEDEF MERKEZE GELINCE SERVO AC KODU 

import cv2
import numpy as np 
#import RPI.GPIO as GPIO
import time


"""
servoPIN1 = 17
servoPIN2 = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN1,GPIO.OUT)
GPIO.setup(servoPIN2,GPIO.OUT)

s1 = GPIO.PWM(servoPIN1, 50)
s2 = GPIO.PWM(servoPIN2, 50)
s1.start(0)
s2.start(0)
"""
def goruntu ():

	cap = cv2.VideoCapture(0)

	while True:
		
		counter = False	

		ret , frame = cap.read()

		if ret == True:

			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_red = np.array([0, 120, 70])
			upper_red = np.array([10, 255, 255])
			mask1 = cv2.inRange(hsv, lower_red, upper_red)
			lower_red = np.array([170, 120, 70])
			upper_red = np.array([180, 255, 255])
			mask2 = cv2.inRange(hsv, lower_red, upper_red)

			mask = mask1 + mask2

			contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

			merkez_noktasi = cv2.circle(frame, (320,480), 7, (0, 0, 255), -1)
			cv2.putText(frame, "MERKEZ",(320,480),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2) 
	                
			for contour in contours:
	                    
				if cv2.contourArea(contour) > 3000:
					c = max(contours, key = cv2.contourArea)
					M = cv2.moments(c)
					cx = int(M['m10'] / M['m00'])
					cy = int(M['m01'] / M['m00'])
					(x, y), radius = cv2.minEnclosingCircle(contour)
					center = (int(x), int(y))
					radius = int(radius)
					cv2.circle(frame, center, radius, (0, 255, 0), 2)
					cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
					cv2.putText(frame, "Merkez "+str(cx)+","+str(cy), (cx - 20, cy - 20),   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
					cv2.line(frame,(cx,cy),(320,480),(255,0,0),1)

	                                   
					if(cx >= 320-radius and cx <= 320+radius and cy >= 220-radius and cy <= 220+radius):
						cv2.putText(frame, "Hedef Ortada", (300,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
						counter=True
						break

			cv2.imshow("Mask", mask)
			cv2.imshow("Frame", frame)

			if (cv2.waitKey(15) & 0xFF == ord('q') or counter == True) :
				cap.release()
				cv2.destroyAllWindows()
				break

		else:
			print("Goruntu alinamiyor!!")
			time.sleep(1)

goruntu()
print("Servo 1 aç!!!!!")
#s1.ChangeDutyCycle(aci) #aci=2 -> 0 derece , aci=12 -> 180 derece
time.sleep(3)

goruntu()
print("Servo 2 aç!!!")
#s1.ChangeDutyCycle(aci)
