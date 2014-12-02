#! /usr/bin/env python
import roslib; roslib.load_manifest('fetch_object')

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from move_base_msgs.msg import MoveBaseGoal
from pr2_common_action_msgs.msg import TuckArmsGoal
from pr2_controllers_msgs.msg import PointHeadGoal
from pr2_controllers_msgs.msg import SingleJointPositionGoal
from std_msgs.msg import Header
import actionlib
import rospy
import smach
import tf

class PrepareNavigationToTable(smach.State):
  """Prepares the robot to move with nothing in hand.

  Specifically, it lowers the base and tucks the arms.
  """

  def __init__(self, torso_client, tuck_arms_client):
    """Constructor.

    Args:
      torso_client: An action client for the PR2 torso.
      tuck_arms_client: An action client for the tuck arms action.
    """
    smach.State.__init__(
      self,
      outcomes=['prepare_nav_success', 'prepare_nav_failure']
    )
    self._torso_client = torso_client
    self._tuck_arms_client = tuck_arms_client

  def execute(self, userdata):
    rospy.loginfo('Executing PrepareNavigationToTable')
    self._torso_client.wait_for_server()
    goal = SingleJointPositionGoal(
      position=0.02, # PR2 can't go below 0.0115 for some reason.
      min_duration=rospy.Duration.from_sec(1),
      max_velocity=1
    )
    self._torso_client.send_goal(goal)
    self._torso_client.wait_for_result() # Torso client returns nothing.

    self._tuck_arms_client.wait_for_server()
    goal = TuckArmsGoal(tuck_left=True, tuck_right=True)
    self._tuck_arms_client.send_goal(goal)
    self._tuck_arms_client.wait_for_result()
    result = self._tuck_arms_client.get_result()
    if result is None:
      return 'prepare_nav_failure'
    elif result.tuck_left and result.tuck_right:
      return 'prepare_nav_success'
    else:
      return 'prepare_nav_failure'

class NavigateToTable(smach.State):
  """Navigates the robot to a table.

  First, the robot navigates to the given goal pose. Once it's done navigating,
  it searches for a table, and then adjusts its position to get right next to
  the table.
  """

  def __init__(self, head_client, nav_client, base_publisher, segment_tabletop,
      goal_pose):
    """Constructor.

    Args:
      head_client: An action client for moving the PR2's head.
      nav_client: An action client for navigating with the PR2.
      base_publisher: A Publisher for the base controller.
      tabletop_segmentation_service: A ServiceProxy for the tabletop segmenter.
      goal_pose: The destination, relative to the map.
    """
    smach.State.__init__(
      self,
      outcomes=['nav_to_table_success', 'nav_to_table_failure']
    )
    self._head_client = head_client
    self._nav_client = nav_client
    # TODO(jstn): unused
    self._base_publisher = base_publisher
    self._segment_tabletop = segment_tabletop
    self._goal_pose = goal_pose
    self._tf = tf.TransformerROS()

  def execute(self, userdata):
    rospy.loginfo('Executing NavigateToTable')

    # Look down.
    self._head_client.wait_for_server()
    goal = PointHeadGoal(
      target=PointStamped(
        header=Header(frame_id='/base_link'),
        point=Point(1, 0, 0.6)
      ),
      pointing_axis=Vector3(0, 0, 1),
      pointing_frame='/head_mount_kinect_rgb_optical_frame',
      min_duration=rospy.Duration.from_sec(0.5),
      max_velocity=1
    )
    self._head_client.send_goal(goal)
    self._head_client.wait_for_result()

    # Move roughly to where the table is.
    self._nav_client.wait_for_server()
    goal = MoveBaseGoal(
      target_pose=PoseStamped(
        header=Header(frame_id='/map'),
        pose=self._goal_pose
      )
    )
    self._nav_client.send_goal(goal)
    self._nav_client.wait_for_result()
    state = self._nav_client.get_state()
    if state != GoalStatus.SUCCEEDED:
      rospy.logerr('Failed to roughly navigate to table.')
      return 'nav_to_table_failure'

    # Find the table and drive up to it.
    try:
      response = self._segment_tabletop()
      rospy.loginfo('Table found: {}'.format(response.table))
      if len(response.table.convex_hull.triangles) == 0:
        rospy.logerr('Couldn\'t find table while navigating.')
        return 'nav_to_table_failure'
      goal = MoveBaseGoal(
        target_pose=PoseStamped(
          header=Header(frame_id=response.table.pose.header.frame_id),
          pose=Pose(
            position=Point(
              response.table.x_min - 0.4,
              (response.table.y_min + response.table.y_max) / 2,
              0
            ),
            orientation=self._goal_pose.orientation
          )
        )
      )
      # TODO(jstn): doesn't really do anything.
      rospy.loginfo('Driving to table, goal={}'.format(goal))
      self._nav_client.wait_for_server()
      self._nav_client.send_goal(goal)
      self._nav_client.wait_for_result() # Navigation returns nothing.
      rospy.loginfo('Drove to table.')
      state = self._nav_client.get_state()
      if state != GoalStatus.SUCCEEDED:
        rospy.logerr('Failed to precisely navigate to table.')
        return 'nav_to_table_failure'

      return 'nav_to_table_success'
    except rospy.ServiceException:
      return 'nav_to_table_failure'
