#!/usr/bin/env python
import roslib
roslib.load_manifest('eecs295_grpK')
import rospy
from fw_wrapper.srv import *
import time

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('aGetIsMotorMovingllcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# init function for the robot position
def initRobotPosition():
	rospy.loginfo('Initializing the robot position...')
	for i in range(1,7):
		setMotorTargetPositionCommand(i, 512)
	setMotorTargetPositionCommand(7, 540)
	setMotorTargetPositionCommand(8, 470)

# A wrapper function for putting specified motors back to initial position
# @ param: a list of motor_ids
def initMotorPosition(motor_ids):
	for motor_id in motor_ids:
		if motor_id in range(1, 7):
			setMotorTargetPositionWrapper(motor_id, 512)
		elif motor_id == 7:
			setMotorTargetPositionWrapper(7, 540)
		else:
			setMotorTargetPositionWrapper(8, 470)

################ setMotorTargetPositionWrapper() ################
# A wrapper function for ensuring the motion gets completed before the next one gets run 
def setMotorTargetPositionWrapper(motor_id, position):
	setMotorTargetPositionCommand(motor_id, position)
	while(getIsMotorMovingCommand(motor_id)):
		rospy.loginfo('motor is moving')

################ danceMotionOne() ################
# Dance Motion 1: Put an arm in front and wave the hands, then repeat the motion with the other arm & hand.
def danceMotionOne():
	initRobotPosition()

	setMotorTargetPositionWrapper(5, 750)
	moveCount = 0

	while(moveCount < 10):
		setMotorTargetPositionWrapper(7, 550)
		setMotorTargetPositionWrapper(7, 650)
		moveCount = moveCount + 1
	initMotorPosition([5, 7])

	setMotorTargetPositionWrapper(6, 350)
	moveCount = 0

	while(moveCount < 10):
		setMotorTargetPositionWrapper(8, 550) # THIS VALUE MIGHT NEED TO CHANGE DEPENDING ON THE DIRECTION 
		setMotorTargetPositionWrapper(8, 650) # THIS TOO 
		moveCount = moveCount + 1
	initMotorPosition([6, 8])

################ danceMotionTwo() ################
# Dance Motion 2: Twist the leg few times, and do it with the other leg as well
# why am I craig
def danceMotionTwo():
	initRobotPosition()

	moveCount = 0

	while(moveCount < 10):
		setMotorTargetPositionWrapper(1, 450)
		setMotorTargetPositionWrapper(1, 574)
		moveCount = moveCount + 1
	initMotorPosition(1)

	moveCount = 0

	while(moveCount < 10):
		setMotorTargetPositionWrapper(2, 450)
		setMotorTargetPositionWrapper(2, 574)
		moveCount = moveCount + 1
	initMotorPosition(2)

################ waveHand() ################
# Hand Wave Motion: Puts the arm up, waves its hands.
def waveHand():
	setMotorTargetPositionWrapper(5, 974)
	motionCount = 0
	while(motionCount < 10):
		setMotorTargetPositionWrapper(7, 550)
		setMotorTargetPositionWrapper(7, 650)
		motionCount = motionCount + 1
		if countWave >= 10:
			initRobotPosition()
			break

################ somethingInFront() ################
# Checks whether something is in front of it
# Returns boolean value if the IR sensor value reads greater than 100 
def somethingInFront():
	if getSensorValue(port) > 100:
		return True
	else:
		return False

################ pauseMotion() ################
# @ param : time_to_pause (int)
# Puts the robot into the init position and pauses it for the specified amount of seconds
def pauseMotion(time_to_pause):
	initRobotPosition()
	time.sleep(time_to_pause)


# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group K Control Node...")
    
    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz

    countWave = 0
    countDance = 0
    waveDone = False
    initDone = False
    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 1

		# Stops all motion when it senses something in front of it // TEST THIS WITH THE NEW FIRMWARE WRAPPER
		if somethingInFront():
			pauseMotion(5)      

		# Initialize
		if not initDone:
			initRobotPosition()
			initDone = True

		# The 'wave hand' motion at the start
		if not waveDone:
			waveHand()
			waveDone = True

		# Dance Motion 1
		if waveDone:
			danceMotionOne()
			danceMotionTwo()
	
	    # sleep to enforce loop rate
    	r.sleep()

