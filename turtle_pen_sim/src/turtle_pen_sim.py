#!/usr/bin/env python
# Python Core Module imports.
import time
import math
# Python additional imports.
import turtle

# ROS Python imports.
import tf
import rospy

# ROS Message imports.
from geometry_msgs.msg import Twist, Pose
from turtle_pen_sim_msgs.msg import PenState

# ROS Service imports.
from turtle_pen_sim_msgs.srv import TwoInts, TwoIntsResponse, Teleport, TeleportResponse
from std_srvs.srv import Empty, EmptyResponse

########## User Editable Variables ##########
# Get namespace for nodes. 
namespace = rospy.get_param("turtle_namespace", "turtle")

# Field dimensions in meters. Can be changed by calling turtle/resize service.
dimensions = [6.4, 4.8]

# Loop rate in Hz.
rate = 30

########## End of User Editable Variables ##########

# Static conversion factor for seconds to nanoseconds.
secondsToNanos = 1E+9

# Static conversion factor for nanoseconds to seconds.
nanosToSeconds = 1/1E+9

# Static conversion factor for meters to pixels.
metersToPixels = 100

# Static conversion factor for pixels to meters.
pixelsToMeters = 0.01

# Width of screen.
width = dimensions[0] * metersToPixels

# Height of screen.
height = dimensions[1] * metersToPixels

# Current linear velocity.
linear_vel = 0

# Current angular velocity.
angular_vel = 0

# Time of last received message. Used to calculate movement 
lastUpdateTime = 0

# Flag for screen update needed.
screenChanged = False
# Flag for screen erase needed.
screenErase = False

# Flag for teleport of turtle needed.
teleportNeeded = False
# New X coordinate for turtle teleport.
newX = 0
# New Y coordinate for turtle teleport.
newY = 0

# Current state of pen. Up is True, Down is False.
currentPenUp = False

# Flag for pen state change needed.
penStateChangeNeeded = False

# New state needed for pen, used when penStateChangeNeeded flag is set.
penUp = False


# Gets current time in nanoseconds.
def getTime():
	return time.time() * secondsToNanos

# Gets scale factor from elapsed time for movement in units/sec.
# e.g. if 2 seconds have passed since last update, movement should be 2 * rate(in unit/sec)
def getTimeScaleFactorFromMsg(recvTime):
	global nanosToSeconds
	global getTime
	global time
	return (getTime() - recvTime) * nanosToSeconds

# Callback for cmd_vel msg.
def callback(data):
	global linear_vel
	global angular_vel
	global lastUpdateTime
	lastUpdateTime = getTime() # Set last update time to now. 
	linear_vel = data.linear.x # Set new linear velocity.
	angular_vel = data.angular.z # Set new angular velocity.

# Update movement of turtle.
def updateMoves():
	global lastUpdateTime
	global linear_vel
	global angular_vel
	sf = getTimeScaleFactorFromMsg(lastUpdateTime) # Get time scale factor. 
	turtle.forward(linear_vel * metersToPixels * sf) # Move forward by linear velocity amount.	
	turtle.left(math.degrees(angular_vel) * sf) # Turn based on angular velocity amount, converted to degrees from radians.
	lastUpdateTime = getTime() # Set last update time as now.

# Publishs pose of turtle.
def publishPose():
	global pixelsToMeters
	global metersToPixels
	msg = Pose() # Initialize new Pose message.
	msg.position.x = turtle.xcor() * pixelsToMeters # Sets message X to turtle X. 
	msg.position.y = turtle.ycor() * pixelsToMeters # Sets message Y to turtle Y.
	msg.position.z = 0 # Message Z is always 0 because the turtle moves in a 2d plane.
	quaternion = tf.transformations.quaternion_from_euler(0, 0, turtle.heading()) # Calculate the quaternion for message from heading.
	msg.orientation.x = quaternion[0] # Set quaternion x from calculated values
	msg.orientation.y = quaternion[1] # Set quaternion y from calculated values
	msg.orientation.z = quaternion[2] # Set quaternion z from calculated values
	msg.orientation.w = quaternion[3] # Set quaternion w from calculated values
	pubPose.publish(msg) # Publish message.

# Handles resize request.
def handle_service_resize(req):
	global screenChanged
	global width
	global height
		
	rospy.logdebug("Received resize request.")
	width = req.a # Get new width.
	height = req.b # Get new height.		
	screenChanged = True # Set screen change flag.
	rospy.logdebug("Screen resized.")	
	return TwoIntsResponse(True, "Successful") # Send success response.

# Handles erase screen request.
def handle_service_erase(req):
	global screenErase
	screenErase = True # Set screen erase flag.
	while screenErase: # Wait for update method to erase screen and remove flag.
		waiting = True
	return EmptyResponse() # Send empty response

# Handles teleport request.
def handle_service_teleport(req):
	global teleportNeeded
	global newX
	global newY
	global penStateChangeNeeded
	global penUp
	global currentPenUp
	initialState = currentPenUp # Get current state of pen.
	if bool(req.pen): # If lift pen during teleport.
		penUp = True # Set new pen state.
		penStateChangeNeeded = True # Set pen change flag.		
		while penStateChangeNeeded: # Wait for update method to change pen state and remove flag.
			waiting = True
	newX = req.x # Set new X.
	newY = req.y # Set new Y.
	teleportNeeded = True # Set teleport flag.
	while teleportNeeded: # Wait for update method to teleport turtle and remove flag.
		waiting = True	
	if bool(req.pen): # If lift pen during teleport.
		penUp = initialState # Set pen back to initial state.
		penStateChangeNeeded = True # Set pen change flag.
		while penStateChangeNeeded: # Wait for update method to change pen state and remove flag.
			waiting = True
	return TeleportResponse(True, "Successful") # Send success response.

# Handle pen up request.
def handle_service_pen_up(req):
	global penStateChangeNeeded
	global penUp

	penUp = True # Set pen to up.
	penStateChangeNeeded = True # Set pen change flag.

	while penStateChangeNeeded: # Wait for update method to change pen state and remove flag.
		waiting = True

	return EmptyResponse() # Send empty response.

# Handle pen down request.
def handle_service_pen_down(req):
	global penStateChangeNeeded
	global penUp

	penUp = False # Set pen to down.
	penStateChangeNeeded = True # Set pen change flag.
	
	while penStateChangeNeeded: # Wait for update method to change pen state and remove flag.
		waiting = True

	return EmptyResponse() # Send empty response.

# Updates screen and processes event flags.
def updateScreen():
	global screenChanged
	global width
	global height
	global screenErase
	global teleportNeeded
	global newX
	global newY
	global currentPenUp
	global penUp
	global penStateChangeNeeded
	currentPenUp = not turtle.isdown() # Get current pen state.
	if screenChanged: # If screen resize needed.
		turtle.setup(width, height) # Resize screen.
		screenChanged = False # Remove flag.
	if screenErase: # If screen erase needed.
		turtle.clearscreen() # Clear screen.
		screenErase = False # Remove flag.
	if teleportNeeded: # If teleport needed.	
		turtle.setx(newX) # Set new X.
		turtle.sety(newY) # Set new Y.
		teleportNeeded = False # Remove flag.
	if penStateChangeNeeded: # If pen state change needed.
		if penUp: # If pen needs to be up.
			turtle.penup() # Set pen up.
		else: # Else pen needs to be down.
			turtle.pendown() # Set pen down.
		penStateChangeNeeded = False # Remove flag.

# Publishes current pen state.
def publishPenState():
	global currentPenUp
	global pubPen	
	msg = PenState() # Initialize new PenState message.
	if currentPenUp: # If pen is up.
		msg.state = msg.UP # Set message state to up.
	else: # Else pen is down.
		msg.state = msg.DOWN # Set message state to down.
	pubPen.publish(msg) # Publish message.

# Main setup and loop.
def main(): 
	global width
	global height
	global pubPose
	global pubPen
	global namespace
	global rate
	startTime = getTime() # Get start time.
	rospy.init_node("turtle_pen_sim", anonymous=True) # Initialize ROS Node.

	rospy.Subscriber("/cmd_vel", Twist, callback) # Subscribe to cmd_vel.

	pubPose = rospy.Publisher(namespace+'/pose', Pose, queue_size=10) # Initialize Pose publisher.
	pubPen = rospy.Publisher(namespace+'/penState', PenState, queue_size=10) # Initialize PenState publisher.
	
	service_resize = rospy.Service(namespace+'/resize', TwoInts, handle_service_resize) # Initialize resize service.

	service_erase = rospy.Service(namespace+'/clear_screen', Empty, handle_service_erase)# Initialize clear screen service.

	service_telepot = rospy.Service(namespace+'/teleport', Teleport, handle_service_teleport) # Initialize teleport service.

	service_pen_up = rospy.Service(namespace+'/pen_up', Empty, handle_service_pen_up) # Initialize pen up service.
	service_pen_down = rospy.Service(namespace+'/pen_down', Empty, handle_service_pen_down) # Initialize pen down service.
	
	turtle.setup(int(width), int(height)) # Setup turtle screen.
	turtle.tracer(0, 0) # Turn off turtle automatic updating. Allows program to call update in main event loop.
	r = rospy.Rate(rate) # Initialize loop rate. Rate is set at top of program.
	while not rospy.is_shutdown(): # Main event loop, run while node is alive.		
		updateMoves() # Update moving the turtle.
		publishPose() # Publish Pose of the turtle.
		publishPenState() # Publish turtle pen state.
		updateScreen() # Update turtle screen.
		turtle.update() # Update turtle.
		r.sleep() # Sleep for remainder of loop time. 

	
if __name__ == "__main__": # If program is main program.
	main() # Run main event loop.
