#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import argparse
from robots_for_recycling.srv import ClassifySrv, GraspSrv

class TaskPlanner:
    def __init__(self, manipulator_type, end_effector):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        self.classify_publisher = rospy.Publisher('/classify', Float64, queue_size=10)
        rospy.Subscriber('/recycle', Float64, self.main)
        self.manipulator_type = manipulator_type # string
        # rospy.Subscriber

        # # instantiate correct Manipulator object
        # if (manipulator_type == "franka"):
        #     self.manipulator = Manipulator.Franka(end_effector) # TODO: check this once manipulator files are known
        # elif (manipulator_type == "cartesian"): 
        #     self.manipulator = Manipulator.Cartesian(end_effector) # TODO: check this once manipulator files are known
        # else: 
        #     # rospy.logerr("Invalid manipulator type - Valid types are 'franka' and 'cartesian'")
        #     rospy.signal_shutdown("Invalid manipulator type - Valid types are 'franka' and 'cartesian'") # kills node if you typo it 

    def main(self, msg):
        # self.classify_publisher.publish(msg)
        objects_detected = True
        while objects_detected:
            rospy.wait_for_service('classify_waste')
            try:
                get_bounding_boxes = rospy.ServiceProxy('classify_waste', ClassifySrv)
                print("here")
                yoloV5_data = get_bounding_boxes()
                # print(f"here are the boxes: {yoloV5_data}")
                # print(f"type is: {type(yoloV5_data)}")
                # print(f"dir is: {dir(yoloV5_data)}")
                # print(f"output: {yoloV5_data.output}")
                # print(f"output.data {yoloV5_data.output.data}")

                if yoloV5_data.output.data == []:
                    objects_detected = False
                    print("breaking")
                    break
            except rospy.ServiceException as e:
                print("Classify Service call failed: %s"%e)
            
            print("done with bboxes, waiting for ")
            rospy.wait_for_service('get_grasps')
            try:
                print("getting grasps")
                find_grasps = rospy.ServiceProxy('get_grasps', GraspSrv)
                bbox_msg = Float64MultiArray()
                bbox_msg.data = yoloV5_data.output.data

                grasp = find_grasps(bbox_msg)
            except rospy.ServiceException as e:
                print("Grasp Service call failed: %s"%e)


            print(f"here are the grasps{grasp}")

            # self.manipulator.pickMove(grasp)

            # # cartesian method should handle moving the conveyor belt to the cartesian robot and then back for further classification and finding best grasp
            # self.manipulator.placeMove(grasp.class_label)
            

            ## Send grasps to mainipulator
            # grasp is formatted as: x y z angle width class_label
            # In switch statement, instantiate subclass of manipulation class based on passed in manipulator argument
            # Also add argument to pass in for type of end effector gripper?
            # Task planner node should also handle talking to conveyor belt to move    




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

    parser.add_argument(
        "-e",
        "--end_effector",
        default=None,
        help="What end effector is on the manipulator you are using?"
    )

    args = parser.parse_args()

    TaskPlanner(args.manipulator_type, args.end_effector).run()