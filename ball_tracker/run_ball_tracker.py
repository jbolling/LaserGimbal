import ball_tracker
import serial
import cv2
from time import sleep

current_servo_x = 90
current_servo_y = 90
cp = .02;

# Initialize Ball Tracking Object
bt = ball_tracker.BallTracker(cap=1, filter_tap=0.3)
bt.set_hsv_hi((178, 255, 255))
bt.set_hsv_lo((127,98, 118))

im_w = bt.cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
im_h = bt.cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
target_x = im_w/2
target_y = im_h/2

# Initialize Serial Port
ser = serial.Serial(port = 'COM10', baudrate=115200,timeout=1)
ser.close()
ser.open()
assert ser.isOpen()
sleep(0.5)
packet = str(current_servo_x) + ' ' + str(current_servo_y) + ';'
ser.write(packet)
sleep(0.5)

for i in range(1000):
	# Get ball location in current frame
	(state, res) = bt.detect_ball(strat='HSV',show_res=True)
	if state != None:
		(x, y, r) = state
		print (x, y, r)
		x_err = int(target_x - x)
		y_err = int(y - target_y)
	else:
		x_err = 0
		y_err = 0
	cv2.imshow('result',res)
	
	# Check for laser measurement
	
	# Perform state estimation update
	
	# Perform p-feedback based on estimated depth to align camera
	current_servo_x += int(x_err*cp)
	current_servo_y += int(y_err*cp)
	
	# Send servo values to arduino
	packet = str(current_servo_x) + ' ' + str(current_servo_y) + ';'
	print packet
	ser.write(packet)
	
	# Wait
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
		
	

bt.release_cap()
ser.close()