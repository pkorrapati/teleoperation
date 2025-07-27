#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16

pulseRate = 1000

class Circadian:
    def __init__(self):        
        self.ns = rospy.get_namespace()
        self.pulse = Int16()
        
        rospy.init_node('circadian')
        
        pulse = rospy.get_param('~pulseRate', default=pulseRate)        

        self.rate = rospy.Rate(pulse)
        self.pulse.data = pulse
        self.isAlive = False

        # Publishers
        self.pub_pulse = rospy.Publisher(self.ns + 'pulse', Int16, queue_size=3)        

    def stay_alive(self):
        if not self.isAlive:
            self.isAlive = True

        while not rospy.is_shutdown():
            self.pub_pulse.publish(self.pulse)
            self.rate.sleep()  

        self.isAlive = False      

if __name__ == '__main__':
    try:
        cNode = Circadian()        
        cNode.stay_alive()

    except rospy.ROSInterruptException:
        pass