#! /usr/bin/env python
import roslib; roslib.load_manifest('fetch_object')

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from move_base_msgs.msg import MoveBaseGoal
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal
from pr2_controllers_msgs.msg import Pr2GripperCommand
from object_manipulation_msgs.msg import GripperTranslation
from object_manipulation_msgs.msg import PickupGoal
from pr2_common_action_msgs.msg import TuckArmsGoal
from pr2_controllers_msgs.msg import PointHeadGoal
from pr2_controllers_msgs.msg import SingleJointPositionGoal
from std_msgs.msg import Header
import actionlib
import rospy
import smach
import tf

def check_success_state(client):
  state = client.get_state()
  return state == GoalStatus.SUCCEEDED

class PrepareNavigationToTable(smach.State):
  """Prepares the robot to move with nothing in hand.

  Specifically, it lowers the base and tucks the arms.
  """

  def __init__(self, torso_client, tuck_arms_client):
    """Constructor.

    Args:
      torso_client: An action client for the PR2 torso.
      tuck_arms_client: An action client for tucking the PR2's arms.
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
    torso_goal = SingleJointPositionGoal(
      # TODO(jstn): We don't actually lower the base so we don't have to wait
      # around as much when testing.
      position=0.2, # PR2 can't go below 0.0115 for some reason.
      min_duration=rospy.Duration.from_sec(1),
      max_velocity=1
    )
    self._torso_client.send_goal(torso_goal)
    self._torso_client.wait_for_result()
    if not check_success_state(self._torso_client):
      rospy.logerr('Failed to lower torso.')
      return 'prepare_nav_failure'

    self._tuck_arms_client.wait_for_server()
    tuck_arms_goal = TuckArmsGoal(tuck_left=True, tuck_right=True)
    self._tuck_arms_client.send_goal(tuck_arms_goal)
    self._tuck_arms_client.wait_for_result()
    if not check_success_state(self._tuck_arms_client):
      rospy.logerr('Failed to tuck arms.')
      return 'prepare_nav_failure'
    return 'prepare_nav_success'

class NavigateToTable(smach.State):
  """Navigates the robot to a table.
  """

  def __init__(self, nav_client, goal_pose):
    """Constructor.

    Args:
      nav_client: An action client for navigating with the PR2.
      goal_pose: The destination, relative to the map.
    """
    smach.State.__init__(
      self,
      outcomes=['nav_to_table_success', 'nav_to_table_failure']
    )
    self._nav_client = nav_client
    self._goal_pose = goal_pose

  def execute(self, userdata):
    rospy.loginfo('Executing NavigateToTable')
    
    # Move roughly to where the table is.
    self._nav_client.wait_for_server()
    move_base_goal = MoveBaseGoal(
      target_pose=PoseStamped(
        header=Header(frame_id='/map'),
        pose=self._goal_pose
      )
    )
    self._nav_client.send_goal(move_base_goal)
    self._nav_client.wait_for_result()
    if not check_success_state(self._nav_client):
      rospy.logerr('Failed to roughly navigate to table.')
      return 'nav_to_table_failure'
    return 'nav_to_table_success'

class PreparePickObject(smach.State):
  """Prepares the robot to pick up an object on a table.

  It raises the base and deploys the arms out in front of it.
  """

  def __init__(self, head_client, torso_client, tuck_arms_client):
    """Constructor.

    Args:
      head_client: An action client for moving the PR2's head.
      torso_client: An action client for the PR2 torso.
      tuck_arms_client: An action client for tucking the PR2's arms.
    """
    smach.State.__init__(
      self,
      outcomes=['prepare_pick_success', 'prepare_pick_failure']
    )
    self._head_client = head_client
    self._torso_client = torso_client
    self._tuck_arms_client = tuck_arms_client

  def execute(self, userdata):
    rospy.loginfo('Executing PreparePickObject')

    # Look down.
    self._head_client.wait_for_server()
    point_head_goal = PointHeadGoal(
      target=PointStamped(
        header=Header(frame_id='/base_link'),
        point=Point(1, 0, 0.6)
      ),
      pointing_axis=Vector3(0, 0, 1),
      pointing_frame='/head_mount_kinect_rgb_optical_frame',
      min_duration=rospy.Duration.from_sec(0.5),
      max_velocity=1
    )
    self._head_client.send_goal(point_head_goal)
    self._head_client.wait_for_result()
    if not check_success_state(self._head_client):
      rospy.logerr('Failed to look down.')
      return 'prepare_pick_failure'

    self._torso_client.wait_for_server()
    # TODO(jstn): raise to a height based on the table's height.
    torso_goal = SingleJointPositionGoal(
      position=0.20,
      min_duration=rospy.Duration.from_sec(1),
      max_velocity=1
    )
    self._torso_client.send_goal(torso_goal)
    self._torso_client.wait_for_result()
    if not check_success_state(self._torso_client):
      rospy.logerr('Failed to raise torso.')
      return 'prepare_pick_failure'

    # TODO(jstn): Move to PickObject if it looks like the arms get in the way
    # too much.
    self._tuck_arms_client.wait_for_server()
    tuck_arms_goal = TuckArmsGoal(tuck_left=True, tuck_right=False)
    self._tuck_arms_client.send_goal(tuck_arms_goal)
    self._tuck_arms_client.wait_for_result()
    if not check_success_state(self._tuck_arms_client):
      rospy.logerr('Failed to untuck right arm.')
      return 'prepare_pick_failure'
    return 'prepare_pick_success'

class PickObject(smach.State):
  """Picks up an object on the table.
  """

  def __init__(self, detect_objects, process_collision_map, pickup_client):
    """Constructor.
    """
    smach.State.__init__(
      self,
      outcomes=['pick_success', 'pick_failure']
    )
    self._detect_objects = detect_objects
    self._process_collision_map = process_collision_map
    self._pickup_client = pickup_client

  def execute(self, userdata):
    rospy.loginfo('Executing PickObject')

    detection_response = None
    try:
      # We don't use the object database. These names are a bit confusing but
      # the values are correct.
      detection_response = self._detect_objects(
        return_models=True,
        return_clusters=False
      )
    except rospy.ServiceException:
      rospy.logerr('Failed to detect objects.')
      return 'pick_failure'

    try:
      collision_response = self._process_collision_map(
        detection_result=detection_response.detection,
        reset_collision_models=True,
        reset_attached_models=True,
        desired_frame='base_link'
      )
    except rospy.ServiceException:
      rospy.logerr('Failed to process collision map.')
      return 'pick_failure'

    if (
        len(collision_response.graspable_objects) == 0
        or len(collision_response.collision_object_names) == 0
    ):
      rospy.logerr('No objects found.')
      return 'pick_failure'

    self._pickup_client.wait_for_server()
    pickup_goal = PickupGoal(
      target=collision_response.graspable_objects[0],
      collision_object_name=collision_response.collision_object_names[0],
      collision_support_surface_name=collision_response.collision_support_surface_name,
      arm_name='right_arm',
      lift=GripperTranslation(
        direction=Vector3Stamped(
          header=Header(frame_id='base_link'),
          vector=Vector3(x=0, y=0, z=1)
        ),
        desired_distance=0.1,
        min_distance=0.05
      ),
      use_reactive_lift=False,
      use_reactive_execution=False
    )
    self._pickup_client.send_goal(pickup_goal)
    self._pickup_client.wait_for_result()
    if not check_success_state(self._pickup_client):
      rospy.logerr('Failed to pick up object.')
      return 'pick_failure'
    return 'pick_success'

class PrepareTransport(smach.State):
  """Prepares the robot to move a held object.
  """

  def __init__(self, tuck_arms_client):
    """Constructor.

    Args:
      tuck_arms_client: An action client for tucking the PR2's arms.
    """
    smach.State.__init__(
      self,
      outcomes=['prepare_transport_success', 'prepare_transport_failure']
    )
    self._tuck_arms_client = tuck_arms_client

  def execute(self, userdata):
    rospy.loginfo('Executing PrepareTransport')

    self._tuck_arms_client.wait_for_server()
    # TODO(jstn): We should try to hold the object level and out to the side
    # instead of tucking the arms.
    tuck_arms_goal = TuckArmsGoal(tuck_left=True, tuck_right=True)
    self._tuck_arms_client.send_goal(tuck_arms_goal)
    self._tuck_arms_client.wait_for_result()
    if not check_success_state(self._tuck_arms_client):
      rospy.logerr('Failed to tuck arms for transport.')
      return 'prepare_transport_failure'
    return 'prepare_transport_success'

class NavigateToDestination(smach.State):
  """Navigates the robot to the destination table.
  """

  def __init__(self, nav_client, goal_pose):
    """Constructor.

    Args:
      nav_client: An action client for navigating with the PR2.
      goal_pose: The destination, relative to the map.
    """
    smach.State.__init__(
      self,
      outcomes=['nav_to_destination_success', 'nav_to_destination_failure']
    )
    self._nav_client = nav_client
    self._goal_pose = goal_pose

  def execute(self, userdata):
    rospy.loginfo('Executing NavigateToDestination')

    # Move roughly to where the destination table is.
    self._nav_client.wait_for_server()
    move_base_goal = MoveBaseGoal(
      target_pose=PoseStamped(
        header=Header(frame_id='/map'),
        pose=self._goal_pose
      )
    )
    self._nav_client.send_goal(move_base_goal)
    self._nav_client.wait_for_result()
    if not check_success_state(self._nav_client):
      rospy.logerr('Failed to roughly navigate to destination table.')
      return 'nav_to_destination_failure'
    return 'nav_to_destination_success'

class Reset(smach.State):
  """Resets the robot to its starting position.
  """

  def __init__(self, nav_client, r_gripper_client, tuck_arms_client, goal_pose):
    """Constructor.

    It opens and closes its gripper to release any objects it's holding. Then,
    it tucks its arms and drives back to the given position.

    Args:
      nav_client: An action client for moving the PR2's base.
      r_gripper_client: An action client for opening the right gripper.
      tuck_arms_client: An action client for tucking the PR2's arms.
      goal_pose: Where to go.
    """
    smach.State.__init__(
      self,
      outcomes=['reset_success', 'reset_failure']
    )
    self._nav_client = nav_client
    self._r_gripper_client = r_gripper_client
    self._tuck_arms_client = tuck_arms_client
    self._goal_pose = goal_pose

  def execute(self, userdata):
    rospy.loginfo('Executing Reset')

    self._r_gripper_client.wait_for_server()
    open_goal = Pr2GripperCommandGoal(
      command=Pr2GripperCommand(position=0.08, max_effort=-1)
    )
    self._r_gripper_client.send_goal(open_goal)
    self._r_gripper_client.wait_for_result()
    if not check_success_state(self._r_gripper_client):
      rospy.logerr('Failed to open right gripper.')
      return 'reset_failure'

    close_goal = Pr2GripperCommandGoal(
      command=Pr2GripperCommand(position=0, max_effort=50)
    )
    self._r_gripper_client.send_goal(close_goal)
    self._r_gripper_client.wait_for_result()
    if not check_success_state(self._r_gripper_client):
      rospy.logerr('Failed to close right gripper.')
      return 'reset_failure'

    self._tuck_arms_client.wait_for_server()
    tuck_arms_goal = TuckArmsGoal(tuck_left=True, tuck_right=True)
    self._tuck_arms_client.send_goal(tuck_arms_goal)
    self._tuck_arms_client.wait_for_result()
    if not check_success_state(self._tuck_arms_client):
      rospy.logerr('Failed to tuck arms.')
      return 'reset_failure'

    self._nav_client.wait_for_server()
    move_base_goal = MoveBaseGoal(
      target_pose=PoseStamped(
        header=Header(frame_id='/map'),
        pose=self._goal_pose
      )
    )
    self._nav_client.send_goal(move_base_goal)
    self._nav_client.wait_for_result()
    if not check_success_state(self._nav_client):
      rospy.logerr('Failed to roughly navigate to starting position.')
      return 'reset_failure'
    return 'reset_success'
