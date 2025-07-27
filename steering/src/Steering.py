#!/usr/bin/env python3

import rospy
from dataclasses import dataclass

from std_msgs.msg import Bool, Int8, Int16, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

MAX_LIN_VEL = 0.2
MAX_ROT_VEL = 0.2 
CMD_VEL_TOPIC = '/cmd_vel'

@dataclass
class AXIS_INDX:
    STEER = 0
    CLUCH = 1
    ACCEL = 2
    BRAKE = 3

# buttons
@dataclass
class BUTTON_INDX:
    UP_SHFT = 4
    DN_SHFT = 5

# gear index
@dataclass
class GEAR_INDX:
    MIN_GEAR_IDX = 0
    MAX_GEAR_IDX = 3

    GEAR_REVERSE = 0 # Reverse
    GEAR_PARK = 1 # Park
    GEAR_ONE = 2 # Slow Speed
    GEAR_TWO = 3 # Fast Speed

# Returns a value between lLim and uLim
def limit(value, lLim, uLim):
    return min(max(lLim, value), uLim)

class Steering:
    def __init__(self):        
        global MAX_LIN_VEL, MAX_ROT_VEL, CMD_VEL_TOPIC
        self.ns = rospy.get_namespace()
        self.vel = Twist()      # Actual control commands
        self.selectedGear = Int8()
        self.throttle = Float32()

        self.gear = GEAR_INDX.GEAR_PARK
        self.selectedGear.data = self.gear
        self.buttonHigh = False

        rospy.init_node('steering', anonymous=True)   

        MAX_LIN_VEL = rospy.get_param('~max_lin_vel', 0.2)
        MAX_ROT_VEL = rospy.get_param('~max_rot_vel', 0.2)
        CMD_VEL_TOPIC = rospy.get_param('~cmd_topic', '/cmd_vel')     
        
        # Subscribers
        self.sub_pulse = rospy.Subscriber(self.ns + 'pulse', Int16, self.mobilize)
        self.sub_input = rospy.Subscriber(self.ns + 'joy', Joy, self.input)       # Subscribe to Joy                
    
        # Publishers
        self.pub_motion = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=3)
        self.pub_gear = rospy.Publisher(self.ns + 'gear', Int8, queue_size=2)
        self.pub_throttle = rospy.Publisher(self.ns + 'throttle', Float32, queue_size=2)

        self.pub_gear.publish(self.selectedGear)

    def mobilize(self, data):
        self.pub_motion.publish(self.vel)            
    
    def input(self, data):        
        if data.buttons[BUTTON_INDX.UP_SHFT] and not self.buttonHigh:
            self.buttonHigh = True
            self.gear = limit(self.gear + 1, GEAR_INDX.MIN_GEAR_IDX, GEAR_INDX.MAX_GEAR_IDX)
            self.selectedGear.data = self.gear
            self.pub_gear.publish(self.selectedGear)
            # self.rev = False                # On upshift, set motion to forward direction

        elif data.buttons[BUTTON_INDX.DN_SHFT] and not self.buttonHigh:
            self.buttonHigh = True
            self.gear = limit(self.gear - 1, GEAR_INDX.MIN_GEAR_IDX, GEAR_INDX.MAX_GEAR_IDX)
            self.selectedGear.data = self.gear
            self.pub_gear.publish(self.selectedGear)
            # self.rev = True                 # On downshift, set motion to reverse direction
        
        elif not data.buttons[BUTTON_INDX.UP_SHFT] and not data.buttons[BUTTON_INDX.DN_SHFT] and self.buttonHigh:
            self.buttonHigh = False

        acc = limit((data.axes[AXIS_INDX.ACCEL] + 1) * 0.5, 0.0, 1.0)
        brk = limit((data.axes[AXIS_INDX.BRAKE] + 1) * 0.5, 0.0, 1.0)
        clt = limit((data.axes[AXIS_INDX.CLUCH] + 1) * 0.5, 0.0, 1.0)

        d = limit(data.axes[AXIS_INDX.STEER] * 1, -1.0, 1.0)

        self.throttle.data = acc

        mulx = 0
        mulz = 0 if (self.gear == GEAR_INDX.GEAR_PARK) else 1

        if self.gear == GEAR_INDX.GEAR_REVERSE:
            mulx = -MAX_LIN_VEL
        elif self.gear == GEAR_INDX.GEAR_ONE:
            mulx = 2 * MAX_LIN_VEL / 3
        elif self.gear == GEAR_INDX.GEAR_TWO:
            mulx = MAX_LIN_VEL

        self.vel.linear.x = mulx * acc
        self.vel.angular.z = mulz * d * MAX_ROT_VEL

        self.pub_throttle.publish(self.throttle)        
        
if __name__ == '__main__':
    try:
        sNode = Steering()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass