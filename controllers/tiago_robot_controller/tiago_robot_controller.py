"""tiago_robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

MAX_SPEED = 10.1523
# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')
motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))
   
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

display = robot.getDevice('display')
       
# Parameters of the epuck :
# r= radius of the wheel
# d= distance between the wheels
# phildot = speed of the left wheel
# phirdot = speed of the right wheel
# xw = x position coordinate of the epuck in the world coordinate in real time
# yw = y position coordinate of the epuck in the world coordinate in real time
# map = use to display the environment map with the lidar info
# kernel = use for the convolution for the configuration space map
# stop_state = to make the robot stop moving and display the configuration map
# markerHeight = height of the marker so it is not detected by the lidar
r = 0.016
d = 2*(0.202)
phildot = 0
phirdot = 0
xw = 0
yw = 0
map = np.zeros((300,300))
kernel= np.ones((32,32))  
angles = np.linspace(1.5728/2,-1.5728/2,667) #Need to change FOV field of Lidar when you do that
stop_state = ""
markerHeight = 0.2

# function to map the world coordinate to the 300x300 grid pixel map of the display
def world2map(xw,yw):
    # Wall4 givs us top-left corner => x: -2.35 ; y: 1.85 => (0,299)
    # Wall6 gives us bottom-left corner => x: -2.35; y: -4.015 =>  (0,0)
    # Wall8 gives us the closest bottom-right corner => x: 1.35; y -4.015 => (0,299)
    # The last one is top-right corner => x: 1.35; y: 1.85 => (299,299)
    if xw < -2.35:
        px = 0
    elif xw > 1.35:
        px = 299
    else:
        px = (xw+2.35)*(300/3.7) 
    if yw < -4.015:
        py = 0
    elif yw > 1.85:
        py = 299
    else:
        py = 299-((yw+4.015)*(300/5.865))
    
    return [int(px),int(py)]

# Trajectory to go around the table as well as the index
WP = [[-1.7,-1.5],[-1.7,0],[-1.2,0.3],[0, 0.3],[0.5, -0.2],[0.55, -2],[0,-3.1],[-1.7, -3.4]]
index = 0

# Getting the marker handle and setting it to the first element of WP
marker = robot.getFromDef("marker").getField("translation")
marker.setSFVec3f([*WP[index],markerHeight])

# Boolean to know if we did one turn around the table
first_turn_done = False

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:   
    # Get position and orientation of the robot with the gps and compass      
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    theta=np.arctan2(compass.getValues()[0],compass.getValues()[1])

    # Calculating the distance and heading error
    rho = np.sqrt((xw-WP[index][0])**2 + (yw-WP[index][1])**2)
    alpha = np.arctan2(WP[index][1]-yw,WP[index][0]-xw) - theta
    
    if alpha > np.pi:
        alpha = alpha-2*np.pi
    if alpha < -np.pi:
        alpha = alpha + 2*np.pi
    
    # For debugging
    #print(rho,theta*180/np.pi, alpha*180/np.pi)
    
    # Checking if we have arrive to the marker
    if rho < 0.37:
        if index < len(WP) and first_turn_done == False :
            index+=1
            marker.setSFVec3f([*WP[index],markerHeight])
        elif index >= 0:
            index -=1
            marker.setSFVec3f([*WP[index],markerHeight])
        else:
            stop_state = "STOP"
    # Checking if we made the first turn.
    if index == len(WP)-1 and first_turn_done == False:
        first_turn_done = True 
    
    # Proportional controller for the robot wheels
    p1 = 3
    p2 = 3
        
    phildot = -alpha*p1 + rho*p2
    phirdot = alpha*p1 + rho*p2
    
    phildot = max(min(phildot,MAX_SPEED),-MAX_SPEED)
    phirdot = max(min(phirdot,MAX_SPEED),-MAX_SPEED)
    
    
    # Transforming the lidar information from robot to world coordinates    
    x_r,y_r = [],[]
    x_w, y_w = [],[]
    
    w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                      [np.sin(theta), np.cos(theta), yw],
                      [0,0,1]])
                      
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges==np.inf] = 100
                
    X_i = np.array([ranges*np.cos(angles), ranges*np.sin(angles), np.ones(len(angles))])
    D = w_T_r @ X_i
    
    # Transforming the lidar information in the world coordinates to the display grid
    for d in D.transpose():
        xpx, xpy = world2map(d[0],d[1])
        if map[xpx,xpy] < 1:
            map[xpx,xpy] += 0.01
        v = int(map[xpx,xpy]*255)
        current_color = v*256**2+v*256+v
        display.setColor(current_color)
        display.drawPixel(xpx,xpy)
          
    # For debugging      
    display.setColor(0xFF0000)
    px,py = world2map(xw,yw)
    display.drawPixel(px,py)
     
    #Printing the configuration space of the robot
    if stop_state =="STOP" :
        phildot, phirdot = 0,0 
        cmap = signal.convolve2d(map,kernel,mode='same')
        cspace = cmap>0.9  
        plt.imshow(cspace)
        plt.show()   
   
    #sending command to the Tiago motors
    motor_left.setVelocity(phildot)
    motor_right.setVelocity(phirdot)
    pass

