#!/usr/bin/env python
 
import rospy
import smach
import smach_ros
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import matplotlib.pyplot as plt
import matplotlib.colors
import numpy as np
from geometry_msgs.msg import Twist
import math
import time


bridge = CvBridge()
salir = 0


pos = Pose()
pos_man =Pose()
pos_ball =Pose()
pos_traqueado=0
img_width=0
distancias=[]
dist_min=0

rastreado=False
posicionado_persona=False
posicionado_pelota=False
colocado=False
Persona=False

cmd = Twist()

# Función que guarda la pose odom del robot
def position(msg):
	global pos
	pos = msg.pose.pose
	
#Función que utiliza la camara para traquear un objeto
def image_callback(img_msg):
	global Persona
	try:
		image = bridge.imgmsg_to_cv2(img_msg,"passthrough")
		rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	except:
		rospy.logerr("CVBridge error: {0}".format(e))
	
	# Convert BGR to HSV
	#rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
	#rgb=image
	if (Persona==False):
		light_color = np.array([0,0,90])
		dark_color = np.array([150,150,255])
	else:
		light_color = np.array([90,90,0])
		dark_color = np.array([255,255,150])

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(rgb, light_color, dark_color)

	# Bitwise-AND mask and original image
	output = cv2.bitwise_and(image,image, mask= mask)

	gray = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
	contours = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] 
	rastrear=False
	x=0
	
	#=================== TRAQUEAR COLOR ==========================
	if len(contours) is not 0:
		areas = [cv2.contourArea(c) for c in contours]
		max_index = np.argmax(areas)
		c=contours[max_index]
		area = cv2.contourArea(c)
		if area > 200:
			M = cv2.moments(c)
			if (M["m00"]==0): M["m00"]=1
			x = int(M["m10"]/M["m00"])
			y = int(M['m01']/M['m00'])
			cv2.circle(image, (x,y), 7, (0,255,0), -1)
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(image, '{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
			nuevoContorno = cv2.convexHull(c)
			cv2.drawContours(image, [nuevoContorno], 0, (255,0,0), 3)
	
	global img_width
	img_width=image.shape[1]
	
	global pos_traqueado
	pos_traqueado=x
	#cv2.imshow('output',output)
	cv2.imshow('image',image)
	
	if cv2.waitKey(1) & 0xFF == 27:
		salir = 1
		

	

##################################################################################################
def dist_callback(msg):
# Inicializamos valores
	valorNan=0.6
	count=0
	grupos=5
	suma=0
	media=[]
	len_grupos=len(msg.ranges)/grupos
	print(msg.angle_min)
	print(msg.angle_max)
	extra=0
	min=5
	zona=0
	# Bucle que calcula la media de cada grupo
	for lectura in msg.ranges:
		count=count+1
		if not math.isnan(lectura):
			if math.isinf(lectura):
				suma+=5
			else:
				if lectura<min and (zona<5): # Detectar mínimo de las medidas frontales
					min=lectura
				
				suma += lectura
		
		else:
			suma += valorNan# Sumar valor predefinido si es un NaN
		if count==int(len_grupos):
			zona+=1
			count=0
			media.append(suma/len_grupos)
			suma=0
	#print(min)
	# Se le da la vuelta al array de media para que al principio esten los de más a la izquierda
	media.reverse()
	global distancias
	distancias=media
	global dist_min
	dist_min=min
		
	

# define state Foo
class Traquear(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Traquear','Acercarse'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Traqueado')
		global cmd
		global img_width
		global pos_traqueado
		width=img_width
		x=pos_traqueado
		out="Traquear"
		rate=rospy.Rate(40)
		if pos_traqueado != 0:
			if x<(width/2+10) and x>(width/2-10):
				cmd.angular.z = 0
				out="Acercarse"
				
			elif  x > (width/2+15): #Izquierda
				cmd.angular.z = max(-1.0*(x-(width/2+5))*0.005,-0.2)
			elif x < (width/2-15): # Derecha
				cmd.angular.z = min(1.0*((width/2-5)-x)*0.005,0.2)
		else:
			cmd.angular.z = 1
			cmd.linear.x = 0
		pub.publish(cmd)
		rate.sleep()
		return out
 
 
# define state Bar
class Acercarse(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Traquear','Terminado','Colocarse','Empujar'])
		self.aux=1
	
	def execute(self, userdata):
		rospy.loginfo('Executing state Acercarse')
        
		global pos_ball
		global pos_man
		global Persona
		global dist_min
		global colocado
		min=dist_min
		rango=1
		rate=rospy.Rate(40)
		if min>rango:#acercarse
			cmd.linear.x = min/10
			out="Traquear"
		else: #Calcular pos del objeto
			cmd.linear.x = 0
			cmd.angular.z = 0
			x=pos.orientation.x
			y=pos.orientation.y
			z=pos.orientation.z
			w=pos.orientation.w
			t3= +2.0 * (w * z + x *y)
			t4 = +1.0 -2.0 * (y*y+z*z)
			angle=math.atan2(t3,t4)
			if (Persona==False):
				pos_man.position.x=pos.position.x+(min)*math.cos(angle)
				pos_man.position.y=pos.position.y+(min)*math.sin(angle)
				pos_man.position.z=pos.position.z
				Persona=True
				out="Traquear"
				time.sleep(0.1)
			else:
				pos_ball.position.x=pos.position.x+(min)*math.cos(angle)
				pos_ball.position.y=pos.position.y+(min)*math.sin(angle)
				pos_ball.position.z=pos.position.z
				out="Colocarse"
				if colocado==True and self.aux==2:
					out="Empujar"
				if math.sqrt(pow(pos_ball.position.x-pos_man.position.x,2)+pow(pos_ball.position.y-pos_man.position.y,2))<1 and self.aux==2:
					out="Terminado"
				self.aux=2
		pub.publish(cmd)
		rate.sleep()
		return out
        
class Colocarse(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Colocarse','Traquear'])
	
	def execute(self, userdata):
		rospy.loginfo('Executing state Colocarse')
        # Inicializamos valores y leemos los argumentos
		global pos
		global pos_ball
		global pos_man
		global cmd
		global colocado
		colocado=False
		pos_x=pos_ball.position.x+(pos_ball.position.x-pos_man.position.x)/math.sqrt(pow(pos_ball.position.x+pos_man.position.x,2)+pow(pos_ball.position.y+pos_man.position.y,2))*0.5
		pos_y=pos_ball.position.y+(pos_ball.position.y-pos_man.position.y)/math.sqrt(pow(pos_ball.position.x+pos_man.position.x,2)+pow(pos_ball.position.y+pos_man.position.y,2))*0.5
			
		rate=rospy.Rate(40)
		# Se calculan los ángulos del robot y destino
		angle_des=math.atan2((pos_y-pos.position.y),(pos_x-pos.position.x))
		x=pos.orientation.x
		y=pos.orientation.y
		z=pos.orientation.z
		w=pos.orientation.w
		t3= +2.0 * (w * z + x *y)
		t4 = +1.0 -2.0 * (y*y+z*z)
		angle=math.atan2(t3,t4)
		
		if abs(angle-angle_des)>3.14159:
			if angle_des<0:
				angle_des+=3.14159*2
			else:
				angle+=3.14159*2
		
		global dist_min
		global distancias
		min=dist_min
		media=distancias
		out="Colocarse"
		rate=rospy.Rate(40)
		# Algoritmo que decide a que velocidad va el robot
		print(media)
		print(pos_x)
		print(pos_y)
		if pos.position.x>pos_x-0.08 and pos.position.x<pos_x+0.08 and pos.position.y>pos_y-0.08 and pos.position.y<pos_y+0.08: # En la posición destino
			cmd.linear.x = 0
			cmd.angular.z = 0
			colocado=True
			out="Traquear"
			
		elif angle<angle_des-2 and min>0.7: #angulo muy pequeño -> Izquierda
			cmd.linear.x = 0
			cmd.angular.z = 0.7
		elif  angle>angle_des+2 and min>0.7: # #angulo muy grande -> Derecha
			cmd.angular.z = -0.7
			cmd.linear.x = 0
		elif angle<angle_des-0.5 and min>0.7: #angulo muy pequeño -> Izquierda
			cmd.linear.x = 0.6
			cmd.angular.z = 0.8
		elif  angle>angle_des+0.5 and min>0.7: # #angulo muy grande -> Derecha
			cmd.angular.z = -0.8
			cmd.linear.x = 0.6
		elif angle<angle_des-0.1 and min>0.7: #angulo muy pequeño -> Izquierda
			cmd.linear.x = 0.5
			cmd.angular.z = 0.5
		elif  angle>angle_des+0.1 and min>0.7: # #angulo muy grande -> Derecha
			cmd.angular.z = -0.5
			cmd.linear.x = 0.5
		
		else: #Busca como pasar
			if min<0.7 :
				cmd.angular.z = 1
				cmd.linear.x = 0
			else: #Palante
				cmd.linear.x = min/10
		pub.publish(cmd)
		rate.sleep()
		return out

class Empujar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Empujar','Traquear'])
	
	def execute(self, userdata):
		rospy.loginfo('Executing state Empujar')
        # Inicializamos valores y leemos los argumentos
		global pos
		global pos_ball
		global cmd
		pos_x=pos_ball.position.x+(pos_ball.position.x-pos_man.position.x)/math.sqrt(pow(pos_ball.position.x-pos_man.position.x,2)+pow(pos_ball.position.y-pos_man.position.y,2))*0.2
		pos_y=pos_ball.position.y+(pos_ball.position.y-pos_man.position.y)/math.sqrt(pow(pos_ball.position.x-pos_man.position.x,2)+pow(pos_ball.position.y-pos_man.position.y,2))*0.2
			
		rate=rospy.Rate(40)
		# Se calculan los ángulos del robot y destino
		angle_des=math.atan2((pos_y-pos.position.y),(pos_x-pos.position.x))
		x=pos.orientation.x
		y=pos.orientation.y
		z=pos.orientation.z
		w=pos.orientation.w
		t3= +2.0 * (w * z + x *y)
		t4 = +1.0 -2.0 * (y*y+z*z)
		angle=math.atan2(t3,t4)
		
		if abs(angle-angle_des)>3.14159:
			if angle_des<0:
				angle_des+=3.14159*2
			else:
				angle+=3.14159*2
		
		global dist_min
		global distancias
		min=dist_min
		media=distancias
		out="Empujar"
		rate=rospy.Rate(40)
		# Algoritmo que decide a que velocidad va el robot
		print(media)
		if pos.position.x>pos_x-0.05 and pos.position.x<pos_x+0.05 and pos.position.y>pos_y-0.05 and pos.position.y<pos_y+0.05: # En la posición destino
			cmd.linear.x = 0
			cmd.angular.z = 0
			print('Destino alcanzado')
			out="Traquear"
			
		elif angle<angle_des-0.1: #angulo muy pequeño -> Izquierda
			cmd.linear.x = 0.5
			cmd.angular.z = 0.2
			#print('Ang_Izquierda')
		elif  angle>angle_des+0.1: # #angulo muy grande -> Derecha
			cmd.angular.z = -0.2
			cmd.linear.x = 0.5
			#print('Ang_Derecha')  
			
		else:
			cmd.angular.z = 0
			cmd.linear.x = 0.5
		pub.publish(cmd)
		rate.sleep()
		return out

#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
 
def main():
	rospy.init_node('Dogebot_busca', anonymous = True)
	
	sub_odom = rospy.Subscriber('/odom', Odometry, position)
	sub_image = rospy.Subscriber("/camera/rgb/image_raw",Image,image_callback)
	sub_dist= rospy.Subscriber('/scan', LaserScan, dist_callback)
 
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['Terminado'])
	
	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('Traquear', Traquear(), 
                        		transitions={'Traquear':'Traquear', 'Acercarse':'Acercarse'})
		smach.StateMachine.add('Acercarse', Acercarse(), 
                        		transitions={'Traquear':'Traquear', 'Terminado':'Terminado','Colocarse':'Colocarse', 'Empujar':'Empujar'})
		smach.StateMachine.add('Colocarse', Colocarse(), 
                        		transitions={'Colocarse':'Colocarse','Traquear':'Traquear'})
		smach.StateMachine.add('Empujar', Empujar(), 
                        		transitions={'Empujar':'Empujar', 'Traquear':'Traquear'})
                        		
	
	# Execute SMACH plan
	outcome = sm.execute()
	
 
 
if __name__ == '__main__':
	main()
 
