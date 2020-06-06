import time
import picamera
import os
import json

iso_vals = [0, 100, 160, 200, 250, 320, 400, 500, 640, 800]
def with_sequence(camera):
	# highest iso for darker environment (more grain, fast shutter speed)
	camera.iso=640
	# Wait for automatic gain control to settle
	time.sleep(2)
	# Now fix the values
	camera.shutter_speed = camera.exposure_speed
	camera.exposure_mode = 'off'
	g = camera.awb_gains
	camera.awb_mode = 'off'
	camera.awb_gains = g
	camera.start_preview()
	time.sleep(10)
	camera.stop_preview()
	camera.capture_sequence(['/home/pi/output/image%02d.jpg' % i for i in range(10)])

	
def with_video(camera):
	camera.exposure_mode = "off"
	camera.shutter_speed = 10000
	camera.start_recording("testvideo.h264", quality = 20)
	camera.wait_recording(10)
	camera.stop_recording()
	
	
if not os.listdir(path='/home/pi/output'):	
	with picamera.PiCamera(resolution = (1640, 1232), framerate=40) as camera:
		with_sequence(camera)
		camera_settings = {'Analog Gain':str(camera.analog_gain), 
							'Digital Gain':str(camera.digital_gain),
							'Exposure Speed':str(camera.exposure_speed),
							'Auto White Balancing Gains':str(camera.awb_gains),
							'ISO':camera.iso}
		json.dump(camera_settings, open("/home/pi/output.json", 'w'))
else:
	quit()
	
