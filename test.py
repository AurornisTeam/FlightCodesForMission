from Aurornis import aurornis 
import argparse
import time
#import keyboard
from Missions import missions


parser = argparse.ArgumentParser()
parser.add_argument('--connect' ,default='/dev/ttyUSB0,115200') #/dev/ttyUSB0,115200

args = parser.parse_args()
    
   
connection_string = args.connect

#-- Create the object
plane = aurornis(connection_string)
mission = missions(plane)


while True:
	secenek = input("\nLUTFEN SECINIZ\n1- Ucus oncesi kontrol\n2-Gorevler Listesi\n3-CIKIS \n----->")

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
		#if keyboard.read_key() == "q":
		#	break
	
	elif secenek == '2':
		gorev_secenek = input("\n1- Hedef Takip\n2-Kordinat Bulma\n3-CIKIS \n ---->")
	
		if gorev_secenek == '1':
			print("\n1. Gorev Baslatiliyor\n")
	
		if gorev_secenek == '2':
			print("\n2. Gorev baslatiliyor\n")
			n_WP,missionList = plane.get_current_mission()
			#gorev classindan gorevi baslaticaz
			print(n_WP)
			time.sleep(2)

		if not plane.is_armed():
			plane.arm_and_takeoff()


		mission.uzerine_uc()

					
		if gorev_secenek == '3':
			break
	elif secenek == '3':
		break 
		
