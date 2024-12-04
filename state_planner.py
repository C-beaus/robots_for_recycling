import rospy
from rospy.node import Node
import smach
import smach_ros
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
# from smach_ros import IntrospectionServer
from graphviz import Digraph

'''
CODE REFERENCED FROM PERSONAL PART IN RBE595-B24-Vision-based Robotic Manipulation
Drafted by Alex Brattstrom, Updated Dec. 4th, 2024 
'''


class StartState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'hold', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Initializing state machine...")
        # Simulate gripper control
        return 'success'

class ConveyorState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['process objects', 'pass objects along', 'stop', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("If objects on conveyor or not. If objects to process or not...")
        # Simulate gripper control
        return 'success'

class StartConveyorState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Starting conveyor...")
        # Simulate gripper control
        return 'success'

class StopConveyorState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Stopping conveyor...")
        # Simulate gripper control
        return 'success'


class TakeImageState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Capturing image...")
        # Simulate capturing an image (use actual topics in your implementation)
        return 'success'


class YoloDetectionState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['objects present', 'no objects', 'failure'], output_keys=['bounding_boxes'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Running YOLO detection...")
        userdata.bounding_boxes = [{"label": "item", "x": 100, "y": 50, "width": 200, "height": 100}]
        self.node.get_logger().info(f"Detected bounding boxes: {userdata.bounding_boxes}")

        if len(userdata.bounding_boxes) > 0:
            self.node.get_logger().info("Outcome: objects present")
            return 'objects present'
        else:
            self.node.get_logger().info("Outcome: no objects")
            return 'no objects'

        # self.node.get_logger().info("Running YOLO detection...")
        # # Simulate bounding box detection (use actual service calls here)
        # userdata.bounding_boxes = [{"label": "item", "x": 100, "y": 50, "width": 200, "height": 100}]
        # return 'success'

        # self.node.get_logger().info("Running YOLO detection...")
        # # Simulate bounding box detection
        # userdata.bounding_boxes = [{"label": "item", "x": 100, "y": 50, "width": 200, "height": 100}]
        
        # # Return a valid outcome
        # if len(userdata.bounding_boxes) > 0:
        #     return 'objects present'
        # else:
        #     return 'no objects'


class ChooseClassState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['bounding_boxes'], output_keys=['chosen_class'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Choosing class of object...")
        # Simulate class selection
        userdata.chosen_class = "item"
        return 'success'
    
class ChooseGripperState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['yale', 'suction_cup', 'finka', 'failure'], input_keys=['bounding_boxes'], output_keys=['chosen_class'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Choosing gripper...")
        # Simulate class selection
        userdata.chosen_class = "item"
        return 'success'


class FrankaState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['bounding_boxes'], output_keys=['chosen_class'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Franka Panda robot...")
        # Simulate class selection
        userdata.chosen_class = "item"
        return 'success'
    

class GantryState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['yale', 'suction_cup', 'failure'], input_keys=['bounding_boxes'], output_keys=['chosen_class'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Gantry Robot...")
        # Simulate class selection
        userdata.chosen_class = "item"
        return 'success'
    
class YaleGripperState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['bounding_boxes'], output_keys=['chosen_class'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("GYale Gripper...")
        # Simulate class selection
        userdata.chosen_class = "item"
        return 'success'

class SuctionGripperState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['bounding_boxes'], output_keys=['chosen_class'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Suction Cup Gripper...")
        # Simulate class selection
        userdata.chosen_class = "item"
        return 'success'


class FindClosestItemState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['chosen_class'], output_keys=['closest_item'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Finding closest item...")
        # Simulate finding the closest item
        userdata.closest_item = {"x": 0.5, "y": 0.2, "z": 0.1}
        return 'success'


class ComputeGraspPointsState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['closest_item'], output_keys=['grasp_points'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Computing grasp points...")
        # Simulate GGCNN grasp computation
        userdata.grasp_points = {"x": 0.5, "y": 0.2, "z": 0.1, "angle": 0.0}
        return 'success'


class MoveToPoseState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['grasp_points'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Moving robot to pose...")
        # Simulate MoveIt2 command
        return 'success'


class CloseGripperState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Closing gripper...")
        # Simulate gripper control
        return 'success'


class MoveToDropPoseState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Moving to drop pose...")
        # Simulate MoveIt2 command
        return 'success'


class OpenGripperState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Opening gripper...")
        # Simulate gripper control
        return 'success'



def generate_graph(state_machine):
    dot = Digraph(comment="State Machine Diagram 2")
    
    # Iterate through the states and transitions
    for state_name, state_data in state_machine.get_children().items():
        dot.node(state_name, state_name)
        
        # Use the state machine's transition table to determine connections
        transitions = state_machine._transitions.get(state_name, {})
        for outcome, next_state in transitions.items():
            if next_state:
                dot.edge(state_name, next_state, label=outcome)

    # Render the state machine diagram
    dot.render("smach_state_machine_2", format="png", cleanup=True)
    print("State machine diagram saved as smach_state_machine.png")



def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('state_planner')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['completed', 'no camera', 'aborted'])

    # Add states to the state machine
    with sm:
        smach.StateMachine.add('START', StartState(node), transitions={'success': 'TAKE_IMAGE', 'hold':'START', 'failure': 'aborted'})

        smach.StateMachine.add('TAKE_IMAGE', TakeImageState(node), transitions={'success': 'YOLO_DETECTION', 'failure': 'no camera'})

        smach.StateMachine.add('CONVEYOR', ConveyorState(node), transitions={'process objects': 'STOP_CONVEYOR', 'pass objects along': 'START_CONVEYOR', 'stop': 'completed', 'failure': 'aborted'})
        smach.StateMachine.add('START_CONVEYOR', StartConveyorState(node), transitions={'success': 'TAKE_IMAGE', 'failure': 'aborted'})
        smach.StateMachine.add('STOP_CONVEYOR', StopConveyorState(node), transitions={'success': 'CHOOSE_GRIPPER', 'failure': 'aborted'})
        
        smach.StateMachine.add('YOLO_DETECTION', YoloDetectionState(node), transitions={'objects present': 'CONVEYOR', 'no objects': 'START_CONVEYOR', 'failure': 'aborted'})
        # smach.StateMachine.add('CHOOSE_CLASS', ChooseClassState(node), transitions={'success': 'FIND_CLOSEST_ITEM', 'failure': 'aborted'})
        
        
        smach.StateMachine.add('CHOOSE_GRIPPER', ChooseGripperState(node), transitions={'suction_cup': 'GANTRY', 'yale': 'GANTRY', 'finka': 'FRANKA', 'failure': 'aborted'})
        
        smach.StateMachine.add('GANTRY', GantryState(node), transitions={'yale': 'YALE', 'suction_cup': 'SUCTION_CUP', 'failure': 'aborted'})
        smach.StateMachine.add('YALE', YaleGripperState(node), transitions={'success': 'FIND_CLOSEST_ITEM', 'failure': 'aborted'})
        smach.StateMachine.add('SUCTION_CUP', SuctionGripperState(node), transitions={'success': 'FIND_CLOSEST_ITEM', 'failure': 'aborted'})

        smach.StateMachine.add('FRANKA', FrankaState(node), transitions={'success': 'FIND_CLOSEST_ITEM', 'failure': 'aborted'})

        smach.StateMachine.add('FIND_CLOSEST_ITEM', FindClosestItemState(node), transitions={'success': 'COMPUTE_GRASP_POINTS', 'failure': 'aborted'})
        smach.StateMachine.add('COMPUTE_GRASP_POINTS', ComputeGraspPointsState(node), transitions={'success': 'MOVE_TO_POSE', 'failure': 'aborted'})
        smach.StateMachine.add('MOVE_TO_POSE', MoveToPoseState(node), transitions={'success': 'CLOSE_GRIPPER', 'failure': 'aborted'})
        smach.StateMachine.add('CLOSE_GRIPPER', CloseGripperState(node), transitions={'success': 'MOVE_TO_DROP_POSE', 'failure': 'aborted'})
        smach.StateMachine.add('MOVE_TO_DROP_POSE', MoveToDropPoseState(node), transitions={'success': 'OPEN_GRIPPER', 'failure': 'aborted'})
        smach.StateMachine.add('OPEN_GRIPPER', OpenGripperState(node), transitions={'success': 'TAKE_IMAGE', 'failure': 'aborted'})

    # # Attach Introspection Server
    # sis = IntrospectionServer('state_machine_server', sm, '/SM_ROOT')
    # sis.start()

    generate_graph(sm)

    # Execute SMACH plan
    outcome = sm.execute()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
