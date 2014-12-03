#! /usr/bin/env python
import roslib; roslib.load_manifest('fetch_object')

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from object_manipulation_msgs.msg import PickupAction
from pr2_common_action_msgs.msg import TuckArmsAction
from pr2_controllers_msgs.msg import PointHeadAction
from pr2_controllers_msgs.msg import Pr2GripperCommandAction
from pr2_controllers_msgs.msg import SingleJointPositionAction
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from tabletop_object_detector.srv import TabletopDetection
import actionlib
import rospy
import smach
import states

def main():
  rospy.init_node('fetch_object_action')

  sm = smach.StateMachine(outcomes=['SUCCESS', 'FAILED_BUT_RESET', 'FAILURE'])
  
  torso_client = actionlib.SimpleActionClient(
    'torso_controller/position_joint_action',
    SingleJointPositionAction
  )
  tuck_arms_client = actionlib.SimpleActionClient(
    'tuck_arms',
    TuckArmsAction
  )
  head_client = actionlib.SimpleActionClient(
    'head_traj_controller/point_head_action',
    PointHeadAction
  )
  nav_client = actionlib.SimpleActionClient(
    'move_base',
    MoveBaseAction
  )
  rospy.wait_for_service('/object_detection')
  detect_objects = rospy.ServiceProxy(
    '/object_detection',
    TabletopDetection
  )
  rospy.wait_for_service('/tabletop_collision_map_processing/tabletop_collision_map_processing')
  process_collision_map = rospy.ServiceProxy(
    '/tabletop_collision_map_processing/tabletop_collision_map_processing',
    TabletopCollisionMapProcessing
  )
  pickup_client = actionlib.SimpleActionClient(
    '/object_manipulator/object_manipulator_pickup',
    PickupAction
  )
  r_gripper_client = actionlib.SimpleActionClient(
    'r_gripper_controller/gripper_action',
    Pr2GripperCommandAction
  )

  # Hard-coded poses for testing.
  STARTING_POSE = Pose(
    position=Point(-1, 0.5, 0),
    orientation=Quaternion(0, 0, 0, 1)
  )
  POSE_A = Pose(
    position=Point(3.86, -0.293, 0),
    orientation=Quaternion(0, 0, -0.045, 0.999)
  )
  POSE_B = Pose(
    position=Point(0.15, -3.57, 0),
    orientation=Quaternion(0, 0, 0.997, 0.028)
  )

  with sm:
    smach.StateMachine.add(
      'PREPARE_NAV',
      states.PrepareNavigationToTable(torso_client, tuck_arms_client),
      transitions={
        'prepare_nav_success': 'NAVIGATE_TO_TABLE',
        'prepare_nav_failure': 'RESET' 
      }
    )
    smach.StateMachine.add(
      'NAVIGATE_TO_TABLE',
      states.NavigateToTable(
        nav_client,
        POSE_A
      ),
      transitions={
        'nav_to_table_success': 'PREPARE_PICK',
        'nav_to_table_failure': 'RESET'
      }
    )
    smach.StateMachine.add(
      'PREPARE_PICK',
      states.PreparePickObject(head_client, torso_client, tuck_arms_client),
      transitions={
        'prepare_pick_success': 'PICK_OBJECT',
        'prepare_pick_failure': 'RESET'
      }
    )
    smach.StateMachine.add(
      'PICK_OBJECT',
      states.PickObject(detect_objects, process_collision_map, pickup_client),
      transitions={
        'pick_success': 'PREPARE_TRANSPORT',
        'pick_failure': 'RESET'
      }
    )
    smach.StateMachine.add(
      'PREPARE_TRANSPORT',
      states.PrepareTransport(tuck_arms_client),
      transitions={
        'prepare_transport_success': 'NAVIGATE_TO_DESTINATION',
        'prepare_transport_failure': 'RESET'
      }
    )
    smach.StateMachine.add(
      'NAVIGATE_TO_DESTINATION',
      states.NavigateToDestination(nav_client, POSE_B),
      transitions={
        'nav_to_destination_success': 'RESET',
        'nav_to_destination_failure': 'RESET'
      }
    )
    smach.StateMachine.add(
      'RESET',
      states.Reset(nav_client, r_gripper_client, tuck_arms_client,
        STARTING_POSE),
      transitions={
        'reset_success': 'FAILED_BUT_RESET',
        'reset_failure': 'FAILURE'
      }
    )
  outcome = sm.execute()
  rospy.loginfo('State machine completed with outcome {}'.format(outcome))

  rospy.spin()

if __name__ == '__main__':
  main()
