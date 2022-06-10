- 👋 Hi, I’m @trinhly123
- 👀 I’m interested in ...
- 🌱 I’m currently learning ...
- 💞️ I’m looking to collaborate on ...
- 📫 How to reach me ...

<!---
trinhly123/trinhly123 is a ✨ special ✨ repository because its `README.md` (this file) appears on your GitHub profile.
You can click the Preview link to take a look at your changes.
--->


import cv2, time
import numpy as np
import rospy, math, os, rospkg

from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Int64
from ar_track_alvar_msgs.msg import AlvarMarkers 
from tf.transformations import euler_from_quaternion

red_color=(0,0,255)
fonts=[cv2.FONT_ITALIC]
ar_viewer_id=-1
a=0
b=0
c=0
d=0
arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0} 
roll, pitch, yaw = 0, 0, 0


bridge = CvBridge()
cv_image = np.empty(shape=[0])
distance = []

threshold_60 = 60
threshold_100 = 60

width_640 = 640
scan_width_200, scan_height_20 = 320, 20
lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10

vertical_430 = 430
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
pixel_threshold_160 = 0.8 * area_width_20 * area_height_10

def callbacka(msg): 
	global arData
	global ar_viewer_id
	for i in msg.markers:
		arData["DX"] = i.pose.pose.position.x
		arData["DY"] = i.pose.pose.position.y 
		arData["DZ"] = i.pose.pose.position.z

		arData["AX"] = i.pose.pose.orientation.x 
		arData["AY"] = i.pose.pose.orientation.y 
		arData["AZ"] = i.pose.pose.orientation.z 
		arData["AW"] = i.pose.pose.orientation.w

		ar_viewer_id=i.id


def callbackm(msg):
	global L_color
	L_color=msg.data
	



def callbackn(msg):
	global R_color
	R_color=msg.data

def callbackb(msg):
	global count
	count=msg.data

def callback(data):
	global distance, motor_msg 
	distance = data.ranges

def img_callback(img_data):
	global cv_image
	global bridge
	cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

def drive(Angle, Speed):
	global pub

	msg = xycar_motor()
	msg.angle = Angle
	msg.speed = Speed

	pub.publish(msg)


def start():
	global pub
	global cv_image
	global a
	global b
	global c
	global d
	global t_end, t_end2, t_end3

	rospy.init_node('auto_driver')
	rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
	rospy.Subscriber('/Left_color', String, callbackm)
	rospy.Subscriber('/Right_color', String, callbackn)
	rospy.Subscriber('/time_count', Int64, callbackb)

	rospy.Subscriber('ar_pose_marker', AlvarMarkers, callbacka)	

	rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1) 
	
	time.sleep(5)
	while not rospy.is_shutdown():

		while not cv_image.size == (640*480*3):
			continue


		frame = cv_image

		if cv2.waitKey(1) & 0xFF == 27:
			break

		

		

		(roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"],arData["AW"]))
		roll = math.degrees(roll) 
		pitch = math.degrees(pitch) 
		yaw = math.degrees(yaw)
			
		#frame_yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        	#frame_yuv[:,:,0] = cv2.equalizeHist(frame_yuv[:,:,0])
        	#frame = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR)
		roi = frame[vertical_430:vertical_430 + scan_height_20, :]
		frame = cv2.rectangle(frame, (0, vertical_430), (width_640 - 1,
			vertical_430 + scan_height_20), (255, 0, 0), 3)
		hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

		lbound = np.array([0, 0, threshold_100], dtype=np.uint8)
		ubound = np.array([131, 255, 255], dtype=np.uint8)

		bin = cv2.inRange(hsv, lbound, ubound)

		view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

		left, right = -1, 641
		
		
		for r in range(rmid_440, width_640 - area_width_20, 1):
			area = bin[row_begin_5:row_end_15, r:r + area_width_20]
			if cv2.countNonZero(area) > pixel_threshold_160:
				right = r
				break


		for l in range(lmid_200, area_width_20, -1):
			area = bin[row_begin_5:row_end_15, l - area_width_20:l]
			if cv2.countNonZero(area) > pixel_threshold_160:
				left = l
				break


		if left != -1:
			lsquare = cv2.rectangle(view,
				(left - area_width_20, row_begin_5),
				(left, row_end_15),
				(0, 255, 0), 3)
		else:
			print("Lost left line")
			
			

		if right != 641:
			rsquare = cv2.rectangle(view,
				(right, row_begin_5),
				(right + area_width_20, row_end_15),
				(0, 255, 0), 3)
		else:
			print("Lost right line")
			
			




		center = (right + left)/2
		shift = center - 320

		Angle = (shift)/3+4
		

		if Angle < -50:
			Angle = -50,
		if Angle > 50:
			Angle = 50



		Text="angle: " + str(Angle)
		ang=str(Angle)
		cv2.putText(frame,Text,(30,30),16,1,red_color,2,cv2.LINE_AA)
		cv2.imshow("origin", frame)
		cv2.imshow("view", view)
		

		Speed = 5
		drive(Angle, Speed)

		imgpub = rospy.Publisher("/line_drive/Angle", Image)
		imgpub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

		ok=0
		for degree in range(60,120): 
			if distance[degree] <= 0.3:
				ok += 1
        	if ok > 3:
            		drive(Angle,0)
            		
		if ok <= 3:
			drive(Angle, Speed)
		
		if c==0 and right< 360 and left>280 and R_color == 'R':
			drive(Angle, 0)
			
	
		if c==0 and right< 360 and left>280 and R_color == 'Y':
			drive(Angle, 0)

		if 0.3<arData["DZ"]<0.55 and ar_viewer_id==0 and a==0:
			t_end=time.time()+10
			a=1

		if a==1 and time.time()<t_end:
			drive(Angle,2)
		
		if 0.6<arData["DZ"]<1 and ar_viewer_id==2 and b==0:
	
			b=2
			drive(Angle, Speed)

		if b==2 and L_color=='R' and count<4 and right==641 and left==-1:
			drive(-40,Speed)
			c=2
			

			
		
		if b==2 and L_color=='G' and count>3 and right==641 and left==-1:
			drive(-40,Speed)
			c=2
			
			
		
		if b==2 and L_color=='Y' and count<4 and right==641 and left==-1:# and time.time()<t_end2:
			drive(40,Speed)
			c=1
			
			
		
		if b==2 and L_color=='R' and count>3 and right==641 and left==-1:# and time.time()<t_end2:
			drive(40,Speed)
			c=1
			
		
		if c==1 and right< 360 and left>280 and R_color == 'R':
			drive(Angle, 0)
			c=3
			

		if c==2 and right< 360 and left>280 and L_color == 'R':
			drive(Angle, 0)
			c=3
			

		if d==0 and 0.9<arData["DZ"]<1.5 and ar_viewer_id==6:
			drive(50,2)
			speed=0
			c=3
			b=5
			print('mode 1')
			if 1.0>arData["DZ"] and ar_viewer_id==6:
				d=2
			

		if d==2 and 1.1>arData["DZ"]>0.3 and ar_viewer_id==6:
			drive(-50,2)
			print('mode 2')
			if arData["DZ"]<0.4 and ar_viewer_id==6:
				d=3
			

		if d==3 and 0.3<arData["DZ"]<1.7 and ar_viewer_id==6:
			drive(0,-4)
			print('mode 3')
			if arData["DZ"]>1.6 and ar_viewer_id==6:
				d=4
			

		if d==4 and ar_viewer_id==6 and 1.0<arData["DZ"]<1.8:
			drive(35,2)
			if arData["DZ"]<1.1 and ar_viewer_id==6:
				d=5

		if d==5 and ar_viewer_id==6 and 0.7<arData["DZ"]<1.1:
			drive(-45,2)
			if arData["DZ"]<0.8 and ar_viewer_id==6:
				d=6

		if d==6 and ar_viewer_id==6  and arData["DZ"]<0.8:
			drive(0,0)


		
				
	cv2.destroyAllWindows()




if __name__ == '__main__':
	start()































