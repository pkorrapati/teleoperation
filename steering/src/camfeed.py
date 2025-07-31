#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import CompressedImage

class CamFeed:
    def __init__(self):        
        self.ns = rospy.get_namespace()
        rospy.init_node('cam_feed')
        
        camTopic = rospy.get_param('~camTopic', default='/cam_node/image_raw/compressed')        
        
        # Subscribers
        self.sub_cam = rospy.Subscriber(camTopic, CompressedImage, self.onView)        
        
        # Publishers
        self.pub_cam = rospy.Publisher(self.ns + 'cam_feed', CompressedImage, queue_size=1)        

    def onView(self, data):
        self.pub_cam.publish(data)

if __name__ == '__main__':
    try:
        cNode = CamFeed()        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass