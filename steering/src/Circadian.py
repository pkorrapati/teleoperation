#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16

pulseRate = 1000

class Circadian:
    def __init__(self, beatsPerSecond=100):
        # Creates a unique node 'motor_cortex' by using anonymous=True
        rospy.init_node('circadian', anonymous=True)

        self.rate = rospy.Rate(beatsPerSecond)
        self.isAlive = False     
        
        self.pulse = Int16()
        self.pulse.data = beatsPerSecond

        # Publishers
        self.pub_rate = rospy.Publisher('/pulse', Int16, queue_size=3)        

    def stay_alive(self):
        if not self.isAlive:
            self.isAlive = True

        while not rospy.is_shutdown():
            self.pub_rate.publish(self.pulse)
            self.rate.sleep()  

        self.isAlive = False      

if __name__ == '__main__':
    try:
        pulse = rospy.get_param('~pulseRate', default=pulseRate)
        cNode = Circadian(pulse)
        cNode.stay_alive()

    except rospy.ROSInterruptException:
        pass