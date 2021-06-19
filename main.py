import sat_convert
from GPS import GPS
from servo import Servo
import pigpio
from tle import TLEManager
import time
import math
from imu import IMU
from ADC import ADC
import numpy as np
from ThingSpeak import ThingSpeakCommunication

MIN_AZIMUTH = 5

if __name__ == "__main__":
	# setup TLE manager to get satellite positions
	tle = TLEManager()  # defaults to the lab location at MQ

	# initialise the thingspeak communication
	thingspeak = ThingSpeakCommunication()

	# connects to the pi
	pi = pigpio.pi('192.168.178.229')
	imu = IMU(pi)
	gps = GPS(pi)
	adc = ADC(pi)

	pi.set_mode(17, pigpio.INPUT)
	pi.set_pull_up_down(17, pigpio.PUD_UP)

	print('trying to get GPS location', end='')
	if gps.lat == 0:
		for i in range(0):
			print('-' if gps.update() else '.', end='')
			if gps.lat != 0:
				break
			time.sleep(0.5)

	if gps.lat != 0:
		print()
		print(f'gps location acquired {gps.lat=}, {gps.long=}, {int(gps.alt)=}')
		tle.update_location(gps.lat, gps.long, int(gps.alt))
	# exit()

	# setup the servos
	s1 = Servo(pi, 24, angle_range=[0, 170], pulse_range=[550, 2000])
	s2 = Servo(pi, 25, angle_range=[30, 170], pulse_range=[550, 2000])
	s1.set_angle(90)
	s2.set_angle(90)
	time.sleep(1)

	sent_last_message = False

	state = 'auto'
	prev_button = False

	end_time = time.time() + 10*60  # run for 30 seconds
	while end_time > time.time():
		button = not pi.read(17)
		if not prev_button and button:
			if state == 'auto':
				state = 'js'
			else:
				state = 'auto'
		print(state)
		prev_button = button
		if tle.get_sat_altaz('iss')[0] > MIN_AZIMUTH:
			print('following the iss!')
			pose = tle.get_sat_altaz('iss')  # get the real iss values
			print(f'{pose=}')
		elif tle.get_sat_altaz('moon')[0] > MIN_AZIMUTH:
			print('following the moon!')
			pose = tle.get_sat_altaz('moon')  # get the real moon values
			print(f'{pose=}')
		elif True:
			print('following the fake moon!')
			pose = 45, 0, 1000000

		# rotate the position of the object based on accel and mag values
		roll, pitch, yaw = imu.get_roll_pitch_yaw()
		# roll, pitch, yaw = imu.get_kalman_rpy()
		yaw -= 90  # because the sensor is placed 90 degrees off
		if yaw > 180:
			yaw -= 360
		if yaw < -180:
			yaw += 360
		print('rpy')
		print(roll, pitch, yaw)
		pose = list(pose)
		pose[1] = -pose[1]
		new = sat_convert.el_az_to_xyz(*pose)
		new = sat_convert.r3(new, (-pitch, roll, yaw))

		# convert the new target position to elevation and azimuth
		nel, naz = sat_convert.xyz_to_el_az(*new)
		nel, naz = math.degrees(nel), math.degrees(naz)

		print(f'{nel=}, {naz=}')

		# calculate servo positions
		a1, a2 = sat_convert.el_az_to_servo_angles(nel, naz)
		print(f'{a2=}, {a1=}')
		if state == 'auto':
			s2.angle_range = [30, 180]
			s1.set_angle(a1)
			s2.set_angle(a2)
		else:
			s2.angle_range = [45, 180]
			adc.update()
			a1, a2 = adc[1], adc[0]
			a1, a2 = np.interp(a1, [0, 1023], [0, 180]), np.interp(a2, [0, 1023], [180, 0])
			s1.set_angle(a1)
			s2.set_angle(a2)
		print()

		if int(time.time()) % 10 == 0 and not sent_last_message:
			print('sending values to thingspeak')

			iss_latlong = tle.get_latlong('iss')
			iss_latlong = f'{iss_latlong[0]},{iss_latlong[1]}'
			# print(iss_latlong)

			xyz = sat_convert.servo_angles_to_camera_line(a1, a2)
			xyz = f'{xyz[0][1]},{xyz[1][1]},{xyz[2][1]}'

			print('getting GPS')
			*gps_latlong, _ = gps.update()
			gps_latlong = f'{gps_latlong[0]},{gps_latlong[1]}'

			values = {'field1': iss_latlong,
					  'field2': f'{nel},{naz}',
					  'field3': f'{xyz}',
					  'field4': f'{int(state == "auto")}',
					  'field5': str(roll),
					  'field6': str(pitch),
					  'field7': str(yaw),
					  'field8': f'{gps_latlong}'}
			print(values)
			thingspeak.send(**values)

		if int(time.time()) % 10 == 0:
			sent_last_message = True
		else:
			sent_last_message = False

		time.sleep(0.05)

	gps.close()
