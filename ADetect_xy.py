import jetson.inference
import jetson.utils
import serial
import time

# load pre-trained model for image classification
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

# initialize camera, 'csi://0' is for a MIPI camera such as the waveshare
camera = jetson.utils.videoSource("csi://0")      # '/dev/video0' for V4L2

# initialize desktop display
display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file

# initialize serial port for Arduino USB connection with certain parameters
arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600)

# intialize class_name as blind when camera is not detecting anything
class_name = 'Blind'

# begins camera stream while loop which will run until the display is closed
while display.IsStreaming():
	img = camera.Capture() #captures instantaneous image
	detections = net.Detect(img) #creates detection object which collects classification and location data
	display.Render(img) #render the image to the display
	display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS())) #shows render speed

# enters into detection data array
	for detection in detections:
		class_name = net.GetClassDesc(detection.ClassID) # obtains class name from labels.txt file (from detection object)
		center = detection.Center # obtains x and y coordinates of center of bounding box
		confidence = detection.Confidence # obtains confidence of classification (0-1.0)
		print(f"Detected '{class_name}' at x={center[0]} y={center[1]} with {confidence} confidence")

# sets conditions for motor control based on bounding box center, classification of image, and confidence of classification
		if (class_name == 'person' and 300 < center[0] < 500 and confidence > 0.6):
			data = 'A'
			arduino.write(data.encode())

		elif (class_name == 'person' and  center[0] < 300 and confidence > 0.6):
			data = 'B'
			arduino.write(data.encode())
		
		elif (class_name == 'person' and 900 > center[0] > 700 and confidence > 0.6):
			data = 'C'	
			arduino.write(data.encode())

		elif (class_name == 'person' and center[0] > 900 and confidence > 0.6):
			data = 'D'	
			arduino.write(data.encode())

		elif (class_name != 'person' or confidence < 0.6): 
			data = 'E'
			arduino.write(data.encode())

		elif (center[0] > 500 and center[0] < 700):
			data = 'E'
			arduino.write(data.encode())

		
arduino.close()
