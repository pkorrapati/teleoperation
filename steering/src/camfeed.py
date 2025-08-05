#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import CompressedImage

class CamFeed:
    def __init__(self):        
        self.ns = rospy.get_namespace()
        rospy.init_node('cam_feed')
        
        camTopic = rospy.get_param('~camTopic', default='/cam_node/image_raw/compressed')
        rearCamTopic = rospy.get_param('~rearCamTopic', default='/usb_cam/image_raw/compressed')        
        
        # Subscribers
        self.sub_cam = rospy.Subscriber(camTopic, CompressedImage, self.onView)        
        self.sub_r_cam = rospy.Subscriber(rearCamTopic, CompressedImage, self.onRearView)        
        
        # Publishers
        self.pub_cam = rospy.Publisher(self.ns + 'cam_feed', CompressedImage, queue_size=1)        
        self.pub_r_cam = rospy.Publisher(self.ns + 'rear_feed', CompressedImage, queue_size=1)        

    def onView(self, data):
        self.pub_cam.publish(data)
    
    def onRearView(self, data):
        self.pub_r_cam.publish(data)

if __name__ == '__main__':
    try:
        cNode = CamFeed()        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass