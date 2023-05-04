#!/usr/bin/env python
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import numpy as np
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from path_planner.srv import *
from tf.transformations import *
from math import pi

pick_box = ["GreenBox", "RedBox","BlueBox"]
place_box = ["DepositBoxGreen","DepositBoxRed","DepositBoxBlue"]



class Planner():
  def __init__(self):
    # Initializing move_it commands and display trajectory ros publisher
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    xarm_group = moveit_commander.MoveGroupCommander("xarm6")
    xgripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    planning_frame = xarm_group.get_planning_frame()
    eef_link = xarm_group.get_end_effector_link()
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
    )
    
    # class variables
    self.robot = robot
    self.scene = scene
    self.xarm_group = xarm_group
    self.xgripper = xgripper
    self.display_trajectory_publisher = display_trajectory_publisher
    self.eef_link = eef_link
    
  def add_boxes(self):
    #Adding the colored boxes 
    scene = self.scene
    targets_state = True
    # Red box
    rbox_pose = geometry_msgs.msg.PoseStamped()
    rbox_pose.header.frame_id = pick_box[0]
    rbox_pose.pose.orientation.w = 1.0
    rbox_pose.pose.position.z = 0
    rbox_name = pick_box[0]
    scene.add_box(rbox_name, rbox_pose, size=(0.06, 0.06, 0.06))
    targets_state = targets_state and self.wait_for_state_update(pick_box[0], box_is_known=True)
    # Blue box
    bbox_pose = geometry_msgs.msg.PoseStamped()
    bbox_pose.header.frame_id = pick_box[1]
    bbox_pose.pose.orientation.w = 2.0
    bbox_pose.pose.position.z = 0
    bbox_name = pick_box[1]
    scene.add_box(bbox_name, bbox_pose, size=(0.06, 0.06, 0.06))
    targets_state = targets_state and self.wait_for_state_update(pick_box[1], box_is_known=True)
    # Green box
    gbox_pose = geometry_msgs.msg.PoseStamped()
    gbox_pose.header.frame_id = pick_box[2]
    gbox_pose.pose.orientation.w = 3.0
    gbox_pose.pose.position.z = 0
    gbox_name = pick_box[2]
    scene.add_box(gbox_name, gbox_pose, size=(0.06, 0.06, 0.06))
    targets_state = targets_state and self.wait_for_state_update(pick_box[2], box_is_known=True)

    return targets_state

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    #Checking update from moveit
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        is_known = box_name in scene.get_known_object_names()
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        rospy.sleep(0.1)
        seconds = rospy.get_time()
    return False



  def goToPose(self,pose_goal):
    #move to position
    xarm_group = self.xarm_group
    xarm_group.set_pose_target(pose_goal)
    xarm_group.go(wait=True)
    xarm_group.stop()
    xarm_group.clear_pose_targets()


  def detachBox(self,box_name):
    #release box from gipper and using planning.py to release the box
    try:
        self._open_grip()
        self.scene.remove_attached_object(self.eef_link, name=box_name)
        self.scene.remove_world_object(box_name)
        attach = rospy.ServiceProxy('AttachObject', AttachObject)
        attach(0, box_name)
        return self.wait_for_state_update(box_name, box_is_known=False, box_is_attached=False)
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


  def attachBox(self,box_name):
    #hold box by gipper and using planning.py to release the box
    try:
        touch_links = self.robot.get_link_names(group="xarm_gripper")
        self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)
        self._close_grip()
        attach = rospy.ServiceProxy('AttachObject', AttachObject)
        attach(1, box_name)
        return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=True)
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False
    
  def _close_grip(self):
    #command to close gripper
    xgripper = self.xgripper
    xgripper_joint_values = xgripper.get_current_joint_values()
    close_joint_value = (pi / 11 + pi / 12) / 2
    for i in range(len(xgripper_joint_values)):
        xgripper_joint_values[i] = close_joint_value
    xgripper.go(xgripper_joint_values, wait=True)
    xgripper.stop()

  def _open_grip(self):
    #command to open the gripper
    xgripper = self.xgripper
    xgripper_joint_values = xgripper.get_current_joint_values()
    open_joint_value = 0.0
    for i in range(len(xgripper_joint_values)):
        xgripper_joint_values[i] = open_joint_value
    xgripper.go(xgripper_joint_values, wait=True)
    xgripper.stop()


class myNode():
  def __init__(self):
    #Initialising ROS node
    rospy.init_node('solution', anonymous=True)
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.box_name = ''
    self.box_is_picked = False
    # wait till service is called
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def main(self):
    self.planner = Planner()
    self.planner.add_boxes()
    while True:
        pick = self.getGoal("pick")
        if pick.status == True and pick.goal != "End":
            # move to box
            self._move_to_box(pick.goal)
            place = self.getGoal("place")
            if (place.status == True and place.goal != "End") or place != None:
                # move to place position
                self._move_to_deposit(place.goal)
            else:
                break
        else:
            break
    rospy.signal_shutdown("Task Completed")

  def _move2goal(self, reference, goal):
      transform = np.dot(reference, goal)
      trans = translation_from_matrix(transform)
      quat_rot = quaternion_from_matrix(transform)
      self._go_to_pose(trans, quat_rot)

  def _go_to_pose(self, trans, quat_rot):
      goal = geometry_msgs.msg.Pose()

      # Positions
      goal.position.x = trans[0]
      goal.position.y = trans[1]
      goal.position.z = trans[2]
      xyz_rpy = list(euler_from_quaternion([1.0, 0.0, 0.0, 0.0]))
      xyz_rpy[2] = euler_from_quaternion(quat_rot)[2]
      xyz_quat = quaternion_from_euler(xyz_rpy[0], xyz_rpy[1], xyz_rpy[2])

      # Orientations
      goal.orientation.x = xyz_quat[0]
      goal.orientation.y = xyz_quat[1]
      goal.orientation.z = xyz_quat[2]
      goal.orientation.w = xyz_quat[3]

      self.planner.goToPose(goal)

  def translation_quaternion(self,trans_mat, quat_mat):
      return np.dot(trans_mat, quat_mat)

  def get_target_position(self,target):
      trans = translation_matrix((
          target.transform.translation.x,
          target.transform.translation.y,
          target.transform.translation.z
      ))
      quat_rot = quaternion_matrix((
          target.transform.rotation.x,
          target.transform.rotation.y,
          target.transform.rotation.z,
          target.transform.rotation.w
      ))
      return self.translation_quaternion(trans, quat_rot)

  def _move_to_box(self, box):
      xarm_pose = self._get_xarm_pose()
      box_pose = self._get_goal_pose(box)
      base2box_pose = np.dot(xarm_pose, box_pose)
      base2box_pose_up = translation_matrix((0, 0, 0.125))
      # Move above box
      self._move2goal(base2box_pose, base2box_pose_up)
      xarm_pose = self._get_xarm_pose()
      box_pose = self._get_goal_pose(box)
      # Move down to box
      self._move2goal(xarm_pose, box_pose)
      xarm_pose = self._get_xarm_pose()
      base2box_up_pose = np.dot(base2box_pose, base2box_pose_up)
      # Pick box
      self.planner.attachBox(box)
      self.box_name = box
      self.box_is_picked = True
      # Move up with box
      self._move2goal(xarm_pose, np.dot(inverse_matrix(xarm_pose), base2box_up_pose))

  def _move_to_deposit(self, deposit):
      xarm_pose = self._get_xarm_pose()
      deposit_pose = self._get_goal_pose(deposit)
      base2deposit_pose = np.dot(xarm_pose, deposit_pose)
      base2deposit_pose_up = translation_matrix((0, 0, 0.125))
      # Move above deposit
      self._move2goal(base2deposit_pose, base2deposit_pose_up)
      # Place cargo
      self.planner.detachBox(self.box_name)
      self.box_name = ''
      self.box_is_picked = False

  def _get_xarm_pose(self):
      return inverse_matrix(self.get_target_position(self.tf_goal("link_base")))

  def _get_goal_pose(self, goal):
      return self.get_target_position(self.tf_goal(goal))

  def getGoal(self,action):
    #chech the right place position
    try:
        getObj = rospy.ServiceProxy('RequestGoal', RequestGoal)
        return getObj(action)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

  def tf_goal(self, goal):
    #using tf2 to retrieve the place position
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      try:
        trans = self.tfBuffer.lookup_transform('link_tcp', goal, rospy.Time())
        # Does it always return?
        return trans
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue



if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
