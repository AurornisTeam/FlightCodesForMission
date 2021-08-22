from Aurornis import aurornis 
import argparse
import time
import keyboard
from Missions import missions


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='tcp:127.0.0.1:5762') #connection string USB olacak '/dev/ttyUSB0'
args = parser.parse_args()
    
   
connection_string = args.connect
  
#-- Create the object
plane = aurornis(connection_string)
mission = missions(plane)

mission.uzerine_uc()

"""
while True:
	secenek = input("\nLUTFEN SECINIZ\n1- Ucus oncesi kontrol\n2-Gorevi Baslat\n3-CIKIS \n----->")

	if secenek == '1':
		print("ARM KONTROL : {}".format(plane.is_armed()))
		print("OTO PILOT MOD : {}".format(plane.get_ap_mode()))
		print("KONUM : {}  {}".format(plane.pos_lat,plane.pos_lon))
		print("GPS : {}".format(plane.vehicle.gps_0))
		print("YER HIZI : {}".format(plane.vehicle.groundspeed))
		print("HAVA Hizi : {}".format(plane.vehicle.airspeed))
		print("EKF : {}".format(plane.vehicle.ekf_ok))
		print("YUSEKLIK RELATIVE : {}".format(plane.pos_alt_rel))
		print("BATARYA : {}".format(plane.vehicle.battery))
		print("THROTTLE : {}".format(plane.throttle))
		#print("Lidar uzakligi {}".format(plane.distance))
		print("\n")
		time.sleep(1)

	
	elif secenek == '2':

		print("\n2. Gorev baslatiliyor\n")
		n_WP,missionList = plane.get_current_mission()
		#gorev classindan gorevi baslaticaz
		print(n_WP)
		time.sleep(2)

		#if not plane.is_armed():
			#plane.arm_and_takeoff()  #ARM EDIP TAKE OFF KOMUTU VERIR

		mission.uzerine_uc()
		break


	elif secenek == '3':
		break 
		"""