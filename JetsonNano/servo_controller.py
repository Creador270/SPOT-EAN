import sys
sys.path.append("..")

from Kinematics.kinematics import initIK, plotKinematics
import numpy as np
from adafruit_servokit import ServoKit
import board
import busio
import time
import logging
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class Controllers:
    """
    This script appears to be controlling a quadruped robot (a robot with four legs) 
    using inverse kinematics. Here's a breakdown of what the script does:

    It imports necessary libraries and modules, including Kinematics.kinematics for 
    inverse kinematics calculations, numpy for numerical operations, and 
    adafruit_servokit for controlling servos.

    It defines a Controllers class that handles the control of the robot's legs.

    The Controllers class has several methods:

    __init__: Initializes two ServoKit instances for controlling the servos, 
    sets the servo offsets (which are used to adjust the zero position of each servo),
    and initializes some other variables.

    getDegreeAngles: Converts the joint angles from radians to degrees.

    angleToServo: Converts the joint angles to servo angles using the servo offsets. 
    It also checks if the calculated servo angles are within the valid range 
    (0-180 degrees).

    getServoAngles: Returns the current servo angles.

    servoRotate: Takes the joint angles as input, converts them to servo angles, 
    and then rotates the servos to these angles.

    In the main part of the script, it calculates the initial joint angles for the
    robot's legs using the initIK function from the Kinematics.kinematics module. 
    It then creates an instance of the Controllers class and uses it to rotate 
    the servos to the calculated angles. Finally, it uses the plotKinematics 
    function from the Kinematics.kinematics module to plot the kinematics of the robot.

    In summary, this script is controlling a quadruped robot by calculating the joint 
    angles needed for the robot's legs to reach certain positions, and then rotating 
    the servos to these angles. The positions of the legs are specified in the 
    legEndpoints array in the main part of the script.
    
    """
    def __init__(self):

        logging.info("Initializing Servos")
        logging.info("Initializing Servos")
        i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        logging.info("Initializing ServoKit")

        self.kit = ServoKit(channels=16, i2c=i2c_bus0)

        # centered position perpendicular to the ground
        #default positions to go back to
        self._servo_offsets = {
            0: 170,  # front_left DOWN
            1: 60,  # front_left MID
            2: 60,   # front_left UP
            4: 10,   # front_right DOWN
            5: 150,  # front_right MID
            6: 150,  # front_right UP
            8: 170,  # back_left DOWN
            9: 60,  # back_left MID
            10: 140, # back_left UP
            12: 10,  # back_right DOWN
            13: 145, # back_right MID
            14: 70   # back_right UP
        }
        
        # self._servo_offsets = {k: 90 for k in range(16)}

        self._val_list = np.zeros(16) #[ x for x in range(12) ]

        # All Angles for Leg 3 * 4 = 12 length
        self._thetas = []

    def getDegreeAngles(self, La):
        # radian to degree
        La *= 180/np.pi
        La = [ [ int(x) for x in y ] for y in La ]

        self._thetas = La

    def angleToServo(self, La):
        self.getDegreeAngles(La)

        # Define the mapping from servo index to joint and direction
        mapping = [
            (0, 2, -1),  # FL Lower
            (1, 1, -1),  # FL Upper
            (2, 0, 1),   # FL Shoulder
            (4, 2, 1),   # FR Lower
            (5, 1, 1),   # FR Upper
            (6, 0, -1),  # FR Shoulder
            (8, 2, -1),  # BL Lower
            (9, 1, -1),  # BL Upper
            (10, 0, -1), # BL Shoulder
            (12, 2, 1),  # BR Lower
            (13, 1, 1),  # BR Upper
            (14, 0, 1)   # BR Shoulder
        ]

        for i, (servo, joint, direction) in enumerate(mapping):
            self._val_list[servo] = self._servo_offsets[servo] + direction * self._thetas[i // 3][joint]
            # self._val_list[servo] = self._thetas[i // 3][joint]
            
    def getServoAngles(self):
        return self._val_list

    def servoRotate(self, thetas):
        
        self.angleToServo(thetas)
        #self.angleToServo(np.zeros((4,3)))
        for x in range(len(self._val_list)):
            
            if x>=0 and x<15:
                # logging.info("Servo: " + str(x) + " Angle: " + str(self._val_list[x]))
                self._val_list[x] = (self._val_list[x])
                # logging.info(self._val_list[x])

                if (self._val_list[x] > 180):
                    logging.info("Over 180!!")
                    self._val_list[x] = 179
                    continue
                if (self._val_list[x] <= 0):
                    logging.info("Under 0!!")
                    self._val_list[x] = 1
                    continue
                # self.kit.servo[x].angle = self._val_list[x]


if __name__=="__main__":
    """
    _The legEndpoints array appears to represent the target positions of the 
    endpoints (or "feet") of the robot's four legs in the robot's coordinate space.

    Each row in the array corresponds to one leg of the robot. The four values 
    in each row represent the x, y, z coordinates and a homogeneous coordinate 
    of the leg's endpoint, respectively.

    Here's a breakdown:

    The first value (e.g., 100 or -100) is the x-coordinate of the leg's endpoint. 
    A positive value means the endpoint is in front of the robot, and a negative 
    value means it's behind the robot.
    The second value (-100) is the y-coordinate of the leg's endpoint. 
    It's negative for all legs, which might mean that the endpoints are always to 
    the left of the robot (assuming the robot's coordinate system is right-handed).
    The third value (e.g., 87.5 or -87.5) is the z-coordinate of the leg's endpoint. 
    A positive value means the endpoint is above the robot, and a negative value 
    means it's below the robot.
    The fourth value (1) is a homogeneous coordinate, which is used for 
    calculations in projective geometry. In this case, it's likely used 
    to make the coordinates compatible with a transformation matrix that 
    includes a translation (movement) component.
    So, for example, the first row [100, -100, 87.5, 1] represents a leg endpoint 
    that is 100 units in front of the robot, 100 units to the left of the robot, 
    and 87.5 units above the robot._
    
    So, based on this code, the order of the legs in the inverse kinematics matrix is:

        Front left
        Front right
        Back left
        Back right
            
    """
    legEndpoints=np.array([[100,-100,87.5,1],
                           [100,-100,-87.5,1],
                           [-100,-100,87.5,1],
                           [-100,-100,-87.5,1]])
    thetas = initIK(legEndpoints) #radians

    controller = Controllers()
    
    # Get radian thetas, transform to integer servo angles
    # then, rotate servos
    controller.servoRotate(thetas)
    logging.info(f"Thetas: {controller._thetas}")
    # Get AngleValues for Debugging
    svAngle = controller.getServoAngles()
    logging.info(svAngle)

    # #plot at the end
    plotKinematics()
    