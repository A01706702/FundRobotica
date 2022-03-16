#!/usr/bin/env python
from operator import truediv
from re import M
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
import geometry_msgs.msg
import time

class Planner():
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.arm_group = moveit_commander.MoveGroupCommander("xarm6")
    self.hand_group = moveit_commander.MoveGroupCommander("xarm_gripper")
    
    self.attach = rospy.ServiceProxy('AttachObject', AttachObject)
    self.scene = moveit_commander.PlanningSceneInterface()

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=0.4):
    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
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
  

  def addObstacles(self):
    #TODO: Add obstables in the world
    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    # RED
    RED_pose = geometry_msgs.msg.PoseStamped()
    RED_pose.header.frame_id = targets[0]
    RED_pose.pose.orientation.w = 1.0
    RED_pose.pose.position.z = 0
    self.scene.add_box(targets[0], RED_pose, size=(0.06, 0.06, 0.06))
    # BLUE
    BLUE_pose = geometry_msgs.msg.PoseStamped()
    BLUE_pose.header.frame_id = targets[1]
    BLUE_pose.pose.orientation.w = 2.0
    BLUE_pose.pose.position.z = 0
    self.scene.add_box(targets[1], BLUE_pose, size=(0.06, 0.06, 0.06))
    # GREEN
    GREEN_pose = geometry_msgs.msg.PoseStamped()
    GREEN_pose.header.frame_id = targets[2]
    GREEN_pose.pose.orientation.w = 3.0
    GREEN_pose.pose.position.z = 0
    self.scene.add_box(targets[2], GREEN_pose, size=(0.06, 0.06, 0.06))
    
    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]
    
    # DEPOSIT GREEN
    DEPOGREEN_pose = geometry_msgs.msg.PoseStamped()
    DEPOGREEN_pose.header.frame_id = boxes[0]
    DEPOGREEN_pose.pose.orientation.w = 3.0
    DEPOGREEN_pose.pose.position.z = 0
    self.scene.add_box(boxes[0], DEPOGREEN_pose, size=(0.359288, 0.171428, 0.109964))
    # DEPOSIT RED
    DEPORED_pose = geometry_msgs.msg.PoseStamped()
    DEPORED_pose.header.frame_id = boxes[1]
    DEPORED_pose.pose.orientation.w = 1.0
    DEPORED_pose.pose.position.z = 0
    self.scene.add_box(boxes[1], DEPORED_pose, size=(0.359288, 0.171428, 0.109964))
    # DEPOSIT BLUE
    DEPOBLUE_pose = geometry_msgs.msg.PoseStamped()
    DEPOBLUE_pose.header.frame_id = boxes[2]
    DEPOBLUE_pose.pose.orientation.w = 2.0
    DEPOBLUE_pose.pose.position.z = 0
    self.scene.add_box(boxes[2], DEPOBLUE_pose, size=(0.359288, 0.171428, 0.109964))
    

    # Checar si existen las cajas
    RED_exists = self.wait_for_state_update(targets[0], box_is_known=True)
    BLUE_exists = self.wait_for_state_update(targets[1], box_is_known=True)
    GREEN_exists = self.wait_for_state_update(targets[2], box_is_known=True)
    SpawnBox = RED_exists and BLUE_exists and GREEN_exists
    
#     DEPORED_exists = self.wait_for_state_update(targets[0], box_is_known=True)
#     DEPOBLUE_exists = self.wait_for_state_update(targets[1], box_is_known=True)
#     DEPOGREEN_exists = self.wait_for_state_update(targets[2], box_is_known=True)
#     SpawnDepo = DEPORED_exists and DEPOBLUE_exists and DEPOGREEN_exists
    return SpawnBox #Devuelve un bool

  def goToPose(self,pose_goal):
    #TODO: Code used to move to a given position using move it
    pos_box= geometry_msgs.msg.Pose()
  
    Tbox_x = pose_goal.transform.translation.x
    Tbox_y = pose_goal.transform.translation.y
    Tbox_z = pose_goal.transform.translation.z
    #New position without crashes
    Tbox_z_NC = Tbox_z + 0.15

    pos_box.position.x = Tbox_x
    pos_box.position.y = Tbox_y
    pos_box.position.z = Tbox_z_NC
    pos_box.orientation.x = 1.0
   
    self.arm_group.set_pose_target(pos_box)
    self.arm_group.go() 

  def detachBox(self,box_name):
    #TODO: Open the gripper and call the service that releases the box
    self.hand_group.set_named_target("open")
    self.hand_group.go() 
    self.attach(False, box_name)
    return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=False, timeout=0.4)

  def attachBox(self,box_name,pose_goal):
    pos_box= geometry_msgs.msg.Pose()
  
    Tbox_x = pose_goal.transform.translation.x
    Tbox_y = pose_goal.transform.translation.y
    Tbox_z = pose_goal.transform.translation.z
    #New position without crashes
    Tbox_z_NC = Tbox_z + 1.5

    pos_box.position.x = Tbox_x
    pos_box.position.y = Tbox_y
    pos_box.position.z = Tbox_z
    pos_box.orientation.x = 1.0

    self.arm_group.set_pose_target(pos_box)
    self.arm_group.go()

    self.hand_group.set_joint_value_target({"drive_joint":0.18})
    self.hand_group.go()

    self.attach(True, box_name)
    self.arm_group.set_pose_target(pos_box)
    self.arm_group.go()
    return self.wait_for_state_update(box_name, box_is_known=False, box_is_attached=True, timeout=0.4)

  def cmBack(self):
    pos_fin = geometry_msgs.msg.Pose()

    arm_posx = 0.37
    arm_posz = 0.25    
    arm_pos_x = 1.0

    pos_fin.position.x = arm_posx
    pos_fin.position.z = arm_posz
    pos_fin.orientation.x = arm_pos_x

    self.arm_group.set_pose_target(pos_fin)
    self.arm_group.go()
    

class myNode():
  def __init__(self):
    rospy.init_node('pick_place', anonymous=True)
    self.rate = rospy.Rate(5)
    #TODO: Initialise ROS and create the service calls

    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

  def getGoal(self,action):
    #TODO:Call the service that will provide you with a suitable target for the movement
    req_goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
    return req_goal(action)

  def tf_goal(self, goal):
     #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    pos_box = None
    rate = rospy.Rate(1.0)
    while not pos_box:
      try:
        pos_box = self.tfBuffer.lookup_transform("link_base", goal , rospy.Time.now(), rospy.Duration(1.0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
    return pos_box


  def main(self):
    #TODO: Main code that contains the aplication by Team 1
    self.planner = Planner()
    self.planner.addObstacles()
    print(self.planner.addObstacles())

    # print(self.tf_goal("RedBox"))
    # print(self.tf_goal("DepositBoxRed"))
    # print(self.tf_goal("BlueBox"))
    # print(self.tf_goal("DepositBoxBlue"))
    # print(self.tf_goal("GreenBox"))
    # print(self.tf_goal("DepositBoxGreen"))

    #First objetive grab the RedBox
    self.getGoal("pick")
    MyBox = self.tf_goal("RedBox")
    self.planner.goToPose(MyBox)
    self.planner.attachBox("RedBox",MyBox)
    #Second objetive place the RedBox in deposit
    self.getGoal("place")
    MyDep = self.tf_goal("DepositBoxRed")
    self.planner.goToPose(MyDep)
    self.planner.detachBox("RedBox")

    #Third objetive grab the BlueBox
    self.getGoal("pick")
    MyBox = self.tf_goal("BlueBox")
    self.planner.goToPose(MyBox)
    self.planner.attachBox("BlueBox",MyBox)
    #Fourth objetive place the BlueBox in deposit
    self.getGoal("place")
    MyDep = self.tf_goal("DepositBoxBlue")
    self.planner.goToPose(MyDep)
    self.planner.detachBox("BlueBox")

    #Fifth objetive grab the GreenBox
    self.getGoal("pick")
    MyBox = self.tf_goal("GreenBox")
    self.planner.goToPose(MyBox)
    self.planner.attachBox("GreenBox",MyBox)
    #Sixth objetive place the GreenBox in deposit
    self.getGoal("place")
    MyDep = self.tf_goal("DepositBoxGreen")
    self.planner.goToPose(MyDep)
    self.planner.detachBox("GreenBox")

    self.planner.cmBack()

    rospy.signal_shutdown("Task Completed")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()
  except rospy.ROSInterruptException:
    pass

