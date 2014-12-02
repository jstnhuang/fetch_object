#! /usr/bin/env python
import roslib; roslib.load_manifest('fetch_object')

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from pr2_common_action_msgs.msg import TuckArmsAction
from pr2_controllers_msgs.msg import PointHeadAction
from pr2_controllers_msgs.msg import SingleJointPositionAction
from tabletop_object_detector.srv import TabletopSegmentation
import actionlib
import rospy
import smach
import states

#from states import AdjustPoseForTabletopManipulation
#from states import DetectTabletopObjects
#from states import ProcessCollisionMap
#from states import PickupObject
#from states import PrepareNavigationToDestination
#from states import NavigateToDestination

def main():
  rospy.init_node('fetch_object_action')

  sm = smach.StateMachine(outcomes=['SUCCESS', 'FAILURE'])
  
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
  rospy.wait_for_service('tabletop_segmentation')
  segment_tabletop = rospy.ServiceProxy(
    'tabletop_segmentation',
    TabletopSegmentation
  )
  base_publisher = rospy.Publisher('/base_controller/command', Twist)

  # Hard-coded poses for testing.
  POSE_A = Pose(
    position=Point(3.86, -0.293, 0),
    orientation=Quaternion(0, 0, -0.045, 0.999)
  )
  POSE_TEST = Pose(
    position=Point(0, 0, 0),
    orientation=Quaternion(0, 0, -0.0614, 0.998)
  )
  POSE_B = Pose(
    position=Point(-0.02, -3.73, 0),
    orientation=Quaternion(0, 0, 0.997, 0.756)
  )

  with sm:
    smach.StateMachine.add(
      'PREPARE_NAV',
      states.PrepareNavigationToTable(torso_client, tuck_arms_client),
      transitions={
        'prepare_nav_success': 'NAVIGATE_TO_TABLE',
        'prepare_nav_failure': 'FAILURE' 
      }
    )
    smach.StateMachine.add(
      'NAVIGATE_TO_TABLE',
      states.NavigateToTable(
        head_client,
        nav_client,
        base_publisher,
        segment_tabletop,
        POSE_A
      ),
      transitions={
        'nav_to_table_success': 'SUCCESS',
        'nav_to_table_failure': 'FAILURE'
      }
    )
  outcome = sm.execute()
  rospy.loginfo('State machine completed with outcome {}'.format(outcome))

  rospy.spin()

if __name__ == '__main__':
  main()
