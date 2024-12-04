#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import argparse
# from robots_for_recycling.srv import ClassifySrv, ClassifySrvResponse, GraspSrv

class TaskPlanner:
    def __init__(self, manipulator_type):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        self.classify_publisher = rospy.Publisher('/classify', Float64, queue_size=10)
        rospy.Subscriber('/recycle', Float64, self.main)
        self.manipulator_type = manipulator_type # string
        # rospy.Subscriber

        # instantiate correct Manipulator object
        if (manipulator_type == "franka"):
            manipulator = Manipulator.Franka() # TODO: check this once manipulator files are known
        elif (manipulator_type == "cartesian"): 
            manipulator = Manipulator.Cartesian() # TODO: check this once manipulator files are known
        else: 
            # rospy.logerr("Invalid manipulator type - Valid types are 'franka' and 'cartesian'")
            rospy.signal_shutdown("Invalid manipulator type - Valid types are 'franka' and 'cartesian'") # kills node if you typo it 
        
        self.manipulator = manipulator

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

        ## choose highest qual grasp
        # bestGrasp = 0
        # bestGraspListPosition = 0
        # thisListPosition = 0
        # for g in grasps:
            
        #     g[1] = thisGrasp

        #     if (thisGrasp >= bestGrasp):
        #         bestGrasp = thisGrasp
        #         bestGraspListPosition = thisListPosition
        #     thisListPosition += 1

        ## Send grasps to mainipulator
        # self.manipulator.pickMove(bestGrasp)

         # while objects are not 0:
             ## recieve classified image with objects, object types, bounding boxes, coordinates
             ## send image to grasp generator
             ## recieve list of grasps, quality values
             # choose highest qual grasp
             # pickmove to highest quality
             # releasemove to recycling box

         # in discord: 
            # manipulator ppl: manipulator is a superclass of the actual arms, what are the attributes + methods in the manipulator superclass?
            # UPLOAD EVERYTHING TO GIT!!!
            # same with grasp planner - confirm attributes + methods




    def run(self):
            rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument(
         "-m",
         "--manipulator_type",
         default=None,
         help="What manipulator do you want to use for recycling?"
    )

    args = parser.parse_args()

    TaskPlanner().run(args.manipulator_type)