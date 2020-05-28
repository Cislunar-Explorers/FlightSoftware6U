import time
import picamera

iso_vals = [0, 100, 160, 200, 250, 320, 400, 500, 640, 800]
def with_sports_mode(camera):
	camera.resolution = (1280, 720)
	camera.framerate = 60
	# Wait for automatic gain control to settle
	time.sleep(2)
	# Obtain fastes shutter speed
	camera.exposure_mode = 'sports'
	#shutter_speed = camera.exposure_speed
	# Lock down exposure values
	#camera.exposure_mode = 'off'
	#camera.iso = 100
	#camera.shutter_speed = shutter_speed
	g = camera.awb_gains
	print(g)
	#camera.awb_mode = 'off'
	#camera.awb_gains = g
	camera.start_preview()
	time.sleep(10)
	camera.stop_preview()
	camera.capture_sequence(['image%02d.jpg' % i for i in range(10)])

	
def with_video(camera):
	camera.exposure_mode = "off"
	camera.shutter_speed = 10000
	camera.start_recording("testvideo.h264", quality = 20)
	camera.wait_recording(10)
	camera.stop_recording()
	
with picamera.PiCamera() as camera:
	with_sports_mode(camera)
	print(camera.shutter_speed)
	print(camera.exposure_speed)
	print(camera.iso)
	
