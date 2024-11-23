import rospy
from std_msgs.msg import Float64

class TaskPlanner:
    def __init__(self):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        rospy.Subscriber('/recycle', Float64, self.main())
        self.classify_publisher = rospy.Publisher('/classify', Float64, queue_size=10)

    def main(self):
         self.classify_publisher.publish(4.0)

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