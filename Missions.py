"""
Kirmiziyi gordugu an gpsini kaydediyor , kordinatlar_alindi_mi True oldugu zaman kamerayı kapatıp kordinatları goreve ekliyor  

"""
from dronekit import LocationGlobalRelative
from Aurornis import aurornis
from pymavlink import mavutil
from scipy.spatial import distance as dist

# from adafruit_servokit import ServoKit 
# kit = ServoKit(channels=16)

import time
import math
import numpy as np 
import psutil
import argparse
import copy
import RPi.GPIO as GPIO
import cv2

dispW=640
dispH=480
flip=2 

camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width= 640 , height= 480 , format=NV12, framerate=20/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

 # SJCAM Kamerada 0 degil 1 olabilir deneyin


#KAMERA KALIBRASYONU
"""
DIM=(1280, 720)
K=np.array([[796.2152689047866, 0.0, 666.7474094441768], [0.0, 802.2658823670218, 379.81656819013784], [0.0, 0.0, 1.0]])
D=np.array([[-0.02544263874037485], [-0.08730606939161767], [0.2367713210309387], [-0.1741615421908499]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
"""

#GORUNTU KAYIT
fourcc = cv2.VideoWriter_fourcc(*'MP4V') # XVID algoritmasını tanımlama
kayit_goruntu_isleme= cv2.VideoWriter("KayitBlur.avi",cv2.VideoWriter_fourcc(*"XVID"),20,(640,480))
kayit_frame = cv2.VideoWriter("KayitFrame.avi",cv2.VideoWriter_fourcc(*"XVID"),20,(640,480))

outputlar = open('outputlar.txt','a')


class missions():

    
    def __init__(self,aurornis):
        self.aurornis = aurornis
        self.nextwaypoint = self.aurornis.vehicle.commands.next


    def get_target_from_bearing(self,original_location, ang, dist, altitude=None):

        if altitude is None:
            altitude = original_location.alt

        dNorth  = dist*math.cos(ang)
        dEast   = dist*math.sin(ang)
        tgt= self.aurornis.get_location_metres(original_location, dNorth, dEast)
        tgt.alt = altitude

        return tgt 


    def ground_course_2_location(self,angle_deg, piksel_uzaklik,altitude=None):

        try:
            distance = piksel_uzaklik*math.sqrt(6.02/(140787.8-37075.58*7.083074**(-4203.17*self.aurornis.pos_alt_rel))) 

        except:
            print("Fonksiyon hesap hatasi!")

        tgt = self.get_target_from_bearing(original_location=self.aurornis.location_current,ang=math.radians(angle_deg), dist=distance,altitude=altitude)

        return(tgt)


    def set_ground_course(self,angle_deg,piksel_uzaklik,altitude=None):

        gelen_target = self.ground_course_2_location(angle_deg,altitude,piksel_uzaklik)
        gelen_target = str(gelen_target).replace("LocationGlobalRelative:","")
        gelen_target = str(gelen_target).replace("lat=","")
        gelen_target= str(gelen_target).replace("lon=","")
        gelen_target = str(gelen_target).replace("alt=","")
        gelen_target = str(gelen_target).split(",")
        
        outputlar.write("\nHedefin kordinatları: \n{}\n".format(gelen_target))
        outputlar.write("\nAnlık kordinatlarımız: {}\n".format(self.aurornis.vehicle.location.global_relative_frame))
        
        #print("\nHedef kordinatları: \n{}\n".format(gelen_target))
        #print("\nAnlık kordinatlarımız: {} \n".format(self.aurornis.vehicle.location.global_relative_frame))

        return gelen_target


    def uzerine_uc(self):

        cap = cv2.VideoCapture(0)

        while True:

            next_wp = self.aurornis.mission.next

            print("Anlik gidilen WP: {} Total WP: {} ".format(next_wp,self.aurornis.mission.count))

            if (next_wp > 2): # Kamera hangi wp ye giderken acilacak onu belirler

                #print("New wp radius set to:"),aurornis.parameters['WP_RADIUS'] #PARAMETRE AYARLAMA
            
                kordinatlar_alindi_mi=False

                ret, frame = cap.read()
                #h,w = frame.shape[:2]

                #undistorted_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


                blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

                hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 120, 70])
                upper_red = np.array([10, 255, 255])
                mask1 = cv2.inRange(hsv, lower_red, upper_red)
                lower_red = np.array([170, 120, 70])
                upper_red = np.array([180, 255, 255])
                mask2 = cv2.inRange(hsv, lower_red, upper_red)

                mask = mask1 + mask2

                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                merkez_noktasi = cv2.circle(blurred_frame, (320,480), 7, (0, 0, 255), -1)
                cv2.putText(blurred_frame, "MERKEZ",(320,480),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2) 
                
                for contour in contours:
                    
                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                    
                    if cv2.contourArea(contour) > 300 :# alan ... den buyukse and len(approx) >= 5
                        cv2.putText(blurred_frame, "Hedef Tespit Edildi!!!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)     
                        c = max(contours, key = cv2.contourArea) #max contour
                        M = cv2.moments(c)
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        center = (int(x), int(y))
                        radius = int(radius)
                        cv2.circle(blurred_frame, center, radius, (0, 255, 0), 2)
                        cv2.circle(blurred_frame, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(blurred_frame, "Merkez "+str(cx)+","+str(cy), (cx - 20, cy - 20),   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.line(blurred_frame,(cx,cy),(320,480),(255,0,0),1)
                        piksel_uzaklik = dist.euclidean((cx,cy), (320,480))

                        aci=math.degrees(math.atan((cx-320)/(480-cy)))
                        hedef_yaw=aci+(self.aurornis.att_heading_deg % 360)

                        hedef_gps = self.set_ground_course(hedef_yaw,self.aurornis.pos_alt_rel,piksel_uzaklik)

                        for i in range(0, len(hedef_gps)):
                            hedef_gps[i] = float(hedef_gps[i])  #hedef gps = enlem,boylam,yukseklik seklinde geliyor!

                        kordinatlar_alindi_mi = True  #GPS HESAPLADIKTAN SONRA TRUE OLUR VE KAMERA KAPANIP WP GOREVE EKLENIR
 

                        if(cx >= 320-radius and cx <= 320+radius and cy >= 220-radius and cy <= 220+radius):
                            #Ekranın tam ortası servo kit komutları eklenicek ama bu suan calismaz cunku bu while gps hesapladiktan sonra bitiyor servo acmayi baska bir fonksiyonda hedefin wp'ine gelip gelmediginin kontrolunu yaparak actirabiliriz
                            print("Ekranın ortasında")

                            #kit.servo[servonun pini].angle = 180 #dereceye gore servoyu hareket ettirir 
                            #kit.servo[servonun pini].angle = 120 #dereceye gore servoyu hareket ettirir 

                #SSH'da imshowlar KAPANACAK
                
                cv2.imshow("Mask", mask)
                cv2.imshow("Frame", frame)
                cv2.imshow("Goruntu Isleme",blurred_frame)

                kayit_frame.write(frame)
                kayit_goruntu_isleme.write(blurred_frame)

                if (cv2.waitKey(15) & 0xFF == ord('q') or kordinatlar_alindi_mi==True) :
                    
                    #Elle Kordinat alip WP ekliyoruz!!!
                    self.aurornis.add_last_waypoint_to_mission(39.7022837,32.7573600,20)
                    self.aurornis.add_last_waypoint_to_mission(39.7021949,32.7578187,20)
                    self.aurornis.add_last_waypoint_to_mission(39.7025581,32.7578294,20)  #BURAYA MP DEN BELIRLEDIGINIZ KORDINATI YAZIN !!!
                    self.aurornis.add_last_waypoint_to_mission(hedef_gps[0],hedef_gps[1],10) #enlem,boylam,yukseklik
                    self.aurornis.add_last_waypoint_to_mission(39.7030678,32.7571589,20)  #BURAYA MP DEN BELIRLEDIGINIZ KORDINATI YAZIN !!!
                    self.aurornis.add_last_waypoint_to_mission(39.7031359,32.7566814,20)
                    self.aurornis.add_last_waypoint_to_mission(39.7027748,32.7566680,20)
                    
                    self.aurornis.add_last_waypoint_to_mission(39.7022837,32.7573600,20)
                    self.aurornis.add_last_waypoint_to_mission(39.7021949,32.7578187,20)
                    self.aurornis.add_last_waypoint_to_mission(39.7025581,32.7578294,20)  #BURAYA MP DEN BELIRLEDIGINIZ KORDINATI YAZIN !!!
                    self.aurornis.add_last_waypoint_to_mission(hedef_gps[0],hedef_gps[1],10) #enlem,boylam,yukseklik
                    self.aurornis.add_last_waypoint_to_mission(39.7030678,32.7571589,20) 
                    self.aurornis.add_last_waypoint_to_mission(39.7031359,32.7566814,20) 
                    self.aurornis.add_last_waypoint_to_mission(39.7027748,32.7566680,20)
                    
                    self.aurornis.add_last_waypoint_to_mission(39.7022795,32.7573600,10)
                    
                    print("Mode changing AUTO")
                    self.aurornis.set_ap_mode("AUTO")
                    break

        cap.release()
        cv2.destroyAllWindows()

    def distance_to_current_waypoint(self):
        """
        Bir sonraki waypointe uzakligi metre cinsinden verir
        ilk waypoint yani home location icin "none" doner!!
        """
        nextwaypoint = self.aurornis.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem=self.aurornis.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = self.get_distance_metres(self.aurornis.vehicle.location.global_relative_frame, targetWaypointLocation)
        return distancetopoint

    def get_distance_metres(self,aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """

        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def yuk_birak(self):

        while True:
            nextwaypoint = self.aurornis.vehicle.commands.next
            uzaklik = self.distance_to_current_waypoint() 

            if nextwaypoint ==11 and uzaklik < 100:
                print("Birinci servo acilir ve yuk birakilir")
                servo1 = 17
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(servo1, GPIO.OUT)

                p1 = GPIO.PWM(servo1, 50) # GPIO 17 for PWM with 50Hz
                p1.start(2.5) # Initialization
                p1.ChangeDutyCycle(2.5)
                time.sleep(0.5)
                p1.ChangeDutyCycle(12)
                p1.stop()
                GPIO.cleanup()
                GPIO.setwarnings(False)

            elif nextwaypoint == 18 and uzaklik < 100:
                print("Ikinci servo acilir ve yuk birakilir")
                servo2 = 27
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(servo2, GPIO.OUT)

                p2 = GPIO.PWM(servo2, 50) # GPIO 17 for PWM with 50Hz
                p2.start(2.5) # Initialization
                p2.ChangeDutyCycle(2.5)
                time.sleep(0.5)
                p2.ChangeDutyCycle(12)
                p2.stop()
                GPIO.cleanup()
                break

            else:
                print("Next wp :{} Uzaklik: {}".format(nextwaypoint,uzaklik))
