import ball_tracker
import serial

# Initialize Ball Tracking Object
bt = ball_tracker.BallTracker(cap=1, filter_tap=0.3)
bt.set_hsv_hi((178, 255, 255))
bt.set_hsv_lo((127,98, 118))

# Initialize Serial Port
ser = serial.Serial(0, baudrate=115200,timeout=1)
ser.close()
ser.open()
assert ser.isOpen()


while True:
	# Get ball location in current frame
	(state, res) = bt.detect_ball(strat='SVM',show_res=True)
	if state != None:
		(x, y, r) = state
		print (x, y, r)
	cv2.imshow('result',res)
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
		
	# Check for laser measurement
	
	# Perform state estimation update
	
	# Perform p-feedback based on estimated depth to align camera
	
	# Send servo values to arduino
	

bt.release_cap()
ser.close()