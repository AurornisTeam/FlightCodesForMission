from Aurornis import aurornis
from pymavlink import mavutil

import time
import math
import numpy as np 
import psutil
import argparse
import copy

import cv2

cap = cv2.VideoCapture(1)


class missions():

    
    def __init__(self,aurornis):
        self.aurornis = aurornis


    def get_target_from_bearing(self, original_location, ang, dist, altitude=None):
        """ Create a TGT request packet located at a bearing and distance from the original point
        
        Inputs:
            ang     - [rad] Angle respect to North (clockwise) 
            dist    - [m]   Distance from the actual location
            altitude- [m]
        Returns:
            location - Dronekit compatible
        """
        
        if altitude is None: altitude = original_location.alt
        
        # print '---------------------- simulate_target_packet'
        dNorth  = dist*math.cos(ang)
        dEast   = dist*math.sin(ang)
        # print "Based on the actual heading of %.0f, the relative target's coordinates are %.1f m North, %.1f m East" % (math.degrees(ang), dNorth, dEast) 
        
        #-- Get the Lat and Lon
        tgt     = self.aurornis.get_location_metres(original_location, dNorth, dEast)
        
        tgt.alt = altitude
        # print "Obtained the following target", tgt.lat, tgt.lon, tgt.alt

        return tgt  



    def ground_course_2_location(self, angle_deg, altitude=None):
        """ Creates a target to aim to in order to follow the ground course
        Input:
            angle_deg   - target ground courseTR
9+

            altitude    - target altitude (default the current)
        
        """
        tgt = self.get_target_from_bearing(original_location=self.aurornis.location_current, 
                                             ang=math.radians(angle_deg), 
                                             dist=self.aurornis.pos_alt_rel,  # Focal length 65mm sjcam ,HEDEFIN BIZE OLAN UZAKLIGI
                                             altitude=altitude)
        return(tgt)

    def set_ground_course(self, angle_deg, altitude=None):
        """ Set a ground course
        
        Input:
            angle_deg   - [deg] target heading
            altitude    - [m]   target altitude (default the current)
        
        """
        
        #-- command the angles directly
        self.aurornis.goto(self.ground_course_2_location(angle_deg, altitude))
        print("Hedefin kordinatları: {} \n".format(self.ground_course_2_location(angle_deg, altitude)))
        print("Anlık kordinatlarımız: {} {} {} \n".format(self.aurornis.pos_lat,self.aurornis.pos_lon,self.aurornis.pos_alt_rel))



    def uzerine_uc(self):
        while True:
            #print("New wp radius set to:"),aurornis.parameters['WP_RADIUS']
            #print(aurornis.att_heading_deg)
            next_wp = self.aurornis.mission.next
            #print ("Current WP: %d of %d "%(aurornis.mission.next, aurornis.mission.count)

            if (next_wp == 1):
                ("Changing mode GUIDED")

                self.aurornis.set_ap_mode("GUIDED") 
        
            kordinatlar_alindi_mi=False
            ret, frame = cap.read()


            blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

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
                if cv2.contourArea(contour) > 3000:  # alan ... den buyukse
                    #ilk gördüğü an gps kaydet
                    c = max(contours, key = cv2.contourArea) #max contour
                    M = cv2.moments(c)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
                    cv2.putText(frame, "Merkez "+str(cx)+","+str(cy), (cx - 20, cy - 20),   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.line(frame,(cx,cy),(320,220),(255,0,0),1)
                    aci = math.atan(cx/cy)                
                    self.set_ground_course(math.degrees(aci),10)
                   """ if (cx >320 ):
                        aci=math.atan((cx-320)/(480-cy))
                        print("Ucagin kordinatlari: %d %d " %(self.aurornis.pos_lat,self.aurornis.pos_lon))
                        #print("Hedefin kordinatlari: %d %d " %(aurornis.pos_))
                        self.set_ground_course(math.degrees(aci),self.aurornis.pos_alt_rel)
                    elif(cx < 320  ):
                         aci=math.atan((cx-320)/(480-cy))
                         #print(360-math.degrees(aci))
                         self.set_ground_course(360-math.degrees(aci),self.aurornis.pos_alt_rel)
"""
                    if(cx >= 320-radius and cx <= 320+radius and cy >= 220-radius and cy <= 220+radius):
                        print("Hedef Tam Ortada!!!")
                        print(self.aurornis.pos_lat,self.aurornis.pos_lon,self.aurornis.pos_alt_rel) 
                        #atis_kordinatlari=aurornis.pos_lat,aurornis.pos_lon,aurornis.pos_alt_rel
                        enlem=self.aurornis.pos_lat
                        boylam=self.aurornis.pos_lon
                        yukseklik=self.aurornis.pos_alt_rel
                        #kordinatlar_alindi_mi=True
                    else:
                        aci=math.degrees(math.atan((cx-320)/(480-cy)))
                        hedef_yaw=aci+plane.att_heading_deg  
                        self.set_ground_course(hedef_yaw,self.aurornis.pos_alt_rel)
                        
            #cv2.imshow("Mask", mask)
            #cv2.imshow("Frame", frame)
            if (cv2.waitKey(15) & 0xFF == ord('q') or kordinatlar_alindi_mi==True) :
                self.aurornis.add_last_waypoint_to_mission(enlem,boylam,yukseklik)
                #self.aurornis.servo_atis(8,1500)
                print("Mode changing AUTO")
                self.aurornis.set_ap_mode("AUTO")
                break    
        cap.release()
        cv2.destroyAllWindows()

