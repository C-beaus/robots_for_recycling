#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
# from robots_for_recycling.srv import ClassifySrv, ClassifySrvResponse, GraspSrv

class TaskPlanner:
    def __init__(self):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        self.classify_publisher = rospy.Publisher('/classify', Float64, queue_size=10)
        rospy.Subscriber('/recycle', Float64, self.main)
        # rospy.Subscriber

    def main(self, msg):
         self.classify_publisher.publish(msg)
        # rospy.wait_for_service('classify_waste')
        # try:
        #      get_bounding_boxes = rospy.ServiceProxy('clasify_waste', ClassifySrv)
        #      yoloV5_data = get_bounding_boxes()
        # except rospy.ServiceException as e:
        #      print("Classify Service call failed: %s"%e)

        # rospy.wait_for_service('get_grasps')
        # try:
        #      find_grasps = rospy.ServiceProxy('get_grasps', GraspSrv)
        #      grasps = find_grasps(yoloV5_data)
        # except rospy.ServiceException as e:
        #      print("Grasp Service call failed: %s"%e)


         # while objects are not 0:
             # recieve classified image with objects, object types, bounding boxes, coordinates
             # send image to grasp generator
             # recieve list of grasps, quality values
             # pickmove to highest quality
             # releasemove to recycling box




    def run(self):
            rospy.spin()

if __name__ == '__main__':
    
    TaskPlanner().run()