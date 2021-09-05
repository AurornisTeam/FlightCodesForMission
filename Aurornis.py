from dronekit import connect , LocationGlobalRelative, Command ,Attitude,Battery,VehicleMode
import math
from pymavlink import mavutil
import time

class aurornis():

	def __init__(self,connection_string = None, vehicle = None):
		#connection string - mavproxy stili bağlanma şekli örneğin tcp:127.0.0.1:5760
		#vehicle - dronekit arac tipi , default kullaniyoruz

		if not vehicle is None:
			self.vehicle = vehicle
			print("Hazir arac kullaniliyor")

		elif not connection_string is None:
			print("Araca baglaniliyor...")
			self._connect(connection_string)
		else:
			raise("HATA : gecerli bir dronekit vehicle yada connection string girilimedi!!!")
			return

		self._setup_listeners()  #telemetri mesajlarini al ve tut, ayrica mavlink ile mesajlar cogaltilabilir

		self.airspeed = 0.0          #hava hızı
		self.groundspeed = 0.0		 #yer hızı
		
		self.pos_lat = 0.0  		 #enlem
		self.pos_lon = 0.0			 #boylam
		self.pos_alt_rel = 0.0	     #yere göre yukseklik
		self.pos_alt_abs = 0.0		 #deniz seviyesi yuksekligi

		self.att_roll_deg = 0.0	     #roll acisi
		self.att_pitch_deg = 0.0     #pitch acisi
		self.att_heading_deg = 0.0   #heading acisi

		self.wind_speed = 0.0        #rüzgar hizi
		self.wind_dir_from_deg = 0.0 #araca gelen rüzgarin derecesi
		self.wind_dir_to_deg = 0.0   #

		self.climb_rate = 0.0
		self.throttle = 0.0

		self.ap_mode = ''

		self.mission = self.vehicle.commands #görev komutları-görevler

		self.parameters = self.vehicle.parameters  #missionplanner parametreleri

		self.location_home = LocationGlobalRelative(0,0,0)
		self.location_current = LocationGlobalRelative(0,0,0)

		self.distance = 0.0   #Lidar uzaklik

		self.mission = self.vehicle.commands




	def _connect(self, connection_string): #Private fonksiyon baska yere import edilemez
		self.vehicle = connect(connection_string , wait_ready = False, heartbeat_timeout = 360)


	def _setup_listeners(self): #private fonksiyon

		if True:
			@self.vehicle.on_message('ATTITUDE')
			def listener(vehicle,name ,message):    
				self.att_roll_deg = math.degrees(message.roll)
				self.att_pitch_deg = math.degrees(message.pitch)
				self.att_heading_deg = math.degrees(message.yaw)%360

			@self.vehicle.on_message('GLOBAL_POSITION_INT')   #--pozisyon ve sürat
			def listener(vehicle,name ,message):
				self.pos_lat = message.lat*1e-7               #enlem
				self.pos_lon = message.lon*1e-7               #boylam
				self.pos_alt_rel = message.relative_alt*1e-3  #yere gore olan yukseklik
				self.pos_alt_abs = message.alt*1e-3           #deniz seviyesi yuksekligi
				self.location_current = LocationGlobalRelative(self.pos_lat, self.pos_lon, self.pos_alt_rel) #anlik konum

			@self.vehicle.on_message('VFR_HUD')               #Head up Displa
			def listener(vehicle, name, message):
				self.airspeed = message.airspeed
				self.groundspeed = message.groundspeed
				self.throttle = message.throttle
				self.climb_rate = message.climb 

			@self.vehicle.on_message('WIND')
			def listener(vehicle, name, message):
				self.wind_speed = message.speed
				self.wind_dir_from_deg = message.direction % 360
				self.wind_dir_to_deg = (self.wind_dir_from_deg + 180) % 360  #ruzgar gelme acisi

			@self.vehicle.on_message('BATTERY')
			def listener(vehicle, name, message):
				self.battery = message.battery

			@self.vehicle.on_message('RAGEFINDER')
			def listener(self, name, message):
				self.distance = message.distance

		print("Baglanti Kuruldu!!!")		
		return (self.vehicle)
		
	def is_armed(self):                              # arm kontrol
		return (self.vehicle.armed)

	def arm(self):                                   # arm etme
		self.vehicle.armed = True

	def disarm(self):                                #disarm etme
		self.vehicle.armed = False

	def set_airspeed(self,speed):                    # havaya göre olan hizi değiştirme
		self.vehicle.airspeed = speed

	def get_ap_mode(self):                           #otopilot mod ogrenme
		self._ap_mode = self.vehicle.mode
		return (self.vehicle.mode)

	def set_ap_mode(self,mode):               #otopilot mod değiştirme
		time_0 = time.time()
		
		try:
			tgt_mode = VehicleMode(mode)
		except:
			return False

		while (self.get_ap_mode() != tgt_mode):
			self.vehicle.mode = tgt_mode
			time.sleep(0.2)
			if time.time() < time_0 + 5:
				return (False)

		return (True)


	def get_current_mission(self):

		print("Gorev indiriliyor!")
		self.download_mission()
		missionList = []
		n_wp = 0
		for wp in self.vehicle.commands:
			missionList.append(wp)
			n_wp += 1

		return n_wp,missionList

	def mission_add_takeoff(self,takeoff_altitude=25,takeoff_pitch=15,heading=None):

		if heading is None:
			heading = self.att_heading_deg

		self.download_mission()

		tmp_mission = list(self.mission)

		print(tmp_mission.count)
		is_mission= False

		if len(tmp_mission) >= 1:
			is_mission = True
			print("Current mission")

			for item in tmp_mission:
				print(item)


			if is_mission and tmp_mission[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
				print("Takeoff already in the mission")

			else:
				print("Takeoff is not in the mission :adding")
				self.clear_mission()
				takeoff_item = Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, takeoff_pitch,  0, 0, heading, 0,  0, takeoff_altitude)
				self.mission.add(takeoff_item)
				for item in tmp_mission:
					self.mission.add(item)

				self.vehicle.flush()
				print("Doneee")

	def clear_mission(self):

		cmds = self.vehicle.commands
		self.vehicle.commands.clear()
		self.vehicle.flush()
		# After clearing the mission you MUST re-download the mission from the vehicle
		# before vehicle.commands can be used again
		# (see https://github.com/dronekit/dronekit-python/issues/230)
		self.mission = self.vehicle.commands
		self.mission.download()
		self.mission.wait_ready()

	def download_mission(self):
		#--- download the mission
		""" Download the current mission from the vehicle.
        
		"""
		self.vehicle.commands.download()
		self.vehicle.commands.wait_ready() # wait until download is complete.  
		self.mission = self.vehicle.commands

	def arm_and_takeoff(self, altitude=25, pitch_deg=12):
		""" Arms the UAV and takeoff
		Planes need a takeoff item in the mission and to be set into AUTO mode. The 
		heading is kept constant
        
		Input:
            altitude    - altitude at which the takeoff is concluded
            pitch_deg   - pitch angle during takeoff
        """
		self.mission_add_takeoff(takeoff_altitude=1.5*altitude, takeoff_pitch=pitch_deg)
		print ("Takeoff mission ready")
       
        
        
		while not self.vehicle.is_armable:
			print("Wait to be armable...")
			time.sleep(1.0)
            
        
        #-- Save home
		while self.pos_lat == 0.0:
			time.sleep(0.5)
			print ("Waiting for good GPS...")
		self.location_home      = LocationGlobalRelative(self.pos_lat,self.pos_lon,0) 
        
		print("Home is saved as "), self.location_home
		print ("Vehicle is Armable: try to arm")
		#self.set_ap_mode("MANUAL")
		n_tries = 0
		while not self.vehicle.armed:
			print("Try to arm...")
			self.arm()
			n_tries += 1
			time.sleep(2.0) 
            
			if n_tries > 5:
				print("!!! CANNOT ARM")
				break
                
        #--- Set to auto and check the ALTITUDE
		if self.vehicle.armed: 
			print ("ARMED")
			self.set_ap_mode("AUTO")
            
			while self.pos_alt_rel <= altitude - 20.0:
				print ("Altitude = %.0f"%self.pos_alt_rel)
				time.sleep(2.0)
                
			print("Altitude reached: set to GUIDED")
            #self.set_ap_mode("GUIDED")
            
			time.sleep(1.0)
            
            #print("Sending to the home")
            #self.vehicle.simple_goto(self.location_home) 
            
		return True


	def get_location_metres(self, original_location, dNorth, dEast, is_global=False):
        
		earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
		dLat = dNorth/earth_radius
		dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
		newlat = original_location.lat + (dLat * 180/math.pi)
		newlon = original_location.lon + (dLon * 180/math.pi)
        
		if is_global:
			return LocationGlobal(newlat, newlon,original_location.alt)    
		else:
			return LocationGlobalRelative(newlat, newlon,original_location.alt) 



	def goto(self, location):

		self.vehicle.simple_goto(location)



	def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
            self,            #--- vehicle object
            wp_Last_Latitude,   #--- [deg]  Target Latitude
            wp_Last_Longitude,  #--- [deg]  Target Longitude
            wp_Last_Altitude):  #--- [m]    Target Altitude
       
        # Get the set of commands from the vehicle
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()

            # Save the vehicle commands to a list
            missionlist=[]
            for cmd in cmds:
                missionlist.append(cmd)

        # Modify the mission as needed. For example, here we change the
            wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                               wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
            missionlist.append(wpLastObject)

        # Clear the current mission (command is sent when we call upload())
            cmds.clear()

        #Write the modified mission and flush to the vehicle
            for cmd in missionlist:
                cmds.add(cmd)
            cmds.upload()
        
            return (cmds.count)
