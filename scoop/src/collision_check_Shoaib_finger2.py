import sys
import copy
import rospy
import moveit_commander
from moveit_commander import planning_scene_interface
import moveit_msgs.msg
import numpy as np
from math import *
import math
from std_msgs.msg import String
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, PlanningSceneComponents, ObjectColor, RobotState
import math3d as m3d
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from sensor_msgs.msg import JointState

global gripper_command_publisher

pose_Go_stone_set=[[0.06942508861078982, 0.6161399845543093], [0.03316417533282356, 0.614281037860155], [0.06733632128390001, 0.6513263616051356], [0.03221678182485681, 0.6516813689938115]]

#print pose_Go_stone_set

#global motion group_manipulator
scene = moveit_commander.PlanningSceneInterface()
scene_pub = rospy.Publisher('/move_group_manipulator/monitored_planning_scene', PlanningScene, queue_size=50)

class StateValidity():
    def __init__(self):
        # subscribe to joint joint states
        rospy.Subscriber("joint_states", JointState, self.jointStatesCB, queue_size=1)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')
        self.robot = moveit_commander.RobotCommander()
        self.joint_states_received = False


    def checkCollision(self):
        '''
        check if robotis in collision
        '''
        if self.getStateValidity().valid:
            return False
        else:
            return True


    def jointStatesCB(self, msg):
        '''
        update robot state
        '''
        self.current_robot_state = self.robot.get_current_state()
        self.joint_states_received = True


    def getStateValidity(self, group_name='UR10', constraints=None):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.current_robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result


    def start_collision_checker(self):
        while not self.joint_states_received:
            rospy.sleep(0.1)
        rospy.loginfo('joint states received! continue')
        return self.checkCollision()

def clean_scene():
    global scene_pub, scene, display_trajectory_trigger_pub
    # Use the planning scene object to add or remove objects //Interface
    REFERENCE_FRAME = '/base'
    p = PlanningScene()
    p.is_diff = True       
    scene.remove_world_object()
    scene_pub.publish(p)
    
def add_environment():
    global scene_pub, scene, display_trajectory_trigger_pub, group_manipulator, pose_Go_stone_set
    REFERENCE_FRAME = '/base'
    p = PlanningScene()
    p.is_diff = True    
    

    # Create a scene publisher to push changes to the scene //PlanningScene

    scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=50)

    # Give each of the scene objects a unique name        
    Ground_id = 'ground'
    Bowl_id = 'bowl'
    Table_id = 'table'

    # add ground into the scene
    pose_Ground = PoseStamped()
    pose_Ground.header.frame_id = REFERENCE_FRAME
    pose_Ground.pose.position.x = -2.5
    pose_Ground.pose.position.y = -2.5
    pose_Ground.pose.position.z = -0.01

    scene.add_mesh(Ground_id, pose_Ground,'/home/terry/catkin_ws/src/ur10_extendable_finger/meshes/ground.stl')

    pose_Bowl = PoseStamped()
    pose_Bowl.header.frame_id = REFERENCE_FRAME
    pose_Bowl.pose.position.x = 0.05
    pose_Bowl.pose.position.y = 0.64
    pose_Bowl.pose.position.z = 0.125
    pose_Bowl.pose.orientation.x = 1
    pose_Bowl.pose.orientation.w = 0
    scene.add_mesh(Bowl_id, pose_Bowl,'/home/terry/catkin_ws/src/ur10_extendable_finger/meshes/Go_bowl.STL')

    pose_Table = PoseStamped()
    pose_Table.header.frame_id = REFERENCE_FRAME
    pose_Table.pose.position.x = -0.1
    pose_Table.pose.position.y = 0.55
    pose_Table.pose.position.z = 0
    scene.add_mesh(Table_id, pose_Table,'/home/terry/catkin_ws/src/ur10_extendable_finger/meshes/table.STL', size=(0.01,0.01,0.01))
    '''
    Go_stone_id=[]
    for i in range(len(pose_Go_stone_set)):
        Go_stone_id.append('go_stone'+str(i+1))
    for i in range(len(pose_Go_stone_set)):
        pose_Go_stone = PoseStamped()
        pose_Go_stone.header.frame_id = REFERENCE_FRAME
        pose_Go_stone.pose.position.x = pose_Go_stone_set[i][0]-0.0115
        pose_Go_stone.pose.position.y = pose_Go_stone_set[i][1]-0.0115
        pose_Go_stone.pose.position.z = 0.127
        scene.add_mesh(Go_stone_id[i], pose_Go_stone,'/home/terry/catkin_ws/src/ur10_extendable_finger/meshes/Go_stone.STL', size=(0.001,0.001,0.001))
    '''
def process_start():
    global gripper_command_publisher, group_manipulator
    global scene_pub, scene, group_manipulator, pose_Go_stone_set, fingertip_position, gripper_orientation, Feasible_solution_exist


    rospy.init_node('scoop', anonymous=True)
    collision_checker_node = StateValidity()
    moveit_commander.roscpp_initialize(sys.argv)
  
    robot = moveit_commander.RobotCommander()
    #print robot.get_current_state()
    group_manipulator = moveit_commander.MoveGroupCommander("Manipulator")
    group_gripper = moveit_commander.MoveGroupCommander("Gripper")

    #set max velocity and acceleration scaler
    group_manipulator.set_max_velocity_scaling_factor(0.5);
    group_manipulator.set_max_acceleration_scaling_factor(0.5);
    group_manipulator.set_planning_time(1)

    #initialize the gripper publisher variable
    gripper_command_publisher = rospy.Publisher('CModelRobotInputRob',String,queue_size=20)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.

    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    print "============ Starting tutorial "
    clean_scene()  
    group_gripper.go([-0.01567*(0)+0.62683,0,0,0,0,0,0,0]) 
    rospy.sleep(2)
    group_manipulator_variable_values = [100.35/ 180 * 3.1415926, -79.68/ 180 * 3.1415926, -123.96/ 180 * 3.1415926, -68.25/ 180 * 3.1415926, 89.33/ 180 * 3.1415926, -79.56/ 180 * 3.1415926]
    group_manipulator.set_joint_value_target(group_manipulator_variable_values)
    plan1 = group_manipulator.plan()
    group_manipulator.execute(plan1)
    pose_target = group_manipulator.get_current_pose().pose
    pose_target.position.y += 0.1
    pose_target.position.z += 0.1
    group_manipulator.set_pose_target(pose_target)
    plan1 = group_manipulator.plan()  
    group_manipulator.execute(plan1)
    group_gripper.go([-0.01567*(10)+0.62683,0,0,-0.03,0,0,0,0]) 
    rospy.sleep(2)
    while True:
        listener = tf.TransformListener()
        listener.waitForTransform('/ee_link', '/fixed_finger_tip', rospy.Time(), rospy.Duration(2.0))
        ee2fingertip_temp = listener.lookupTransform('/ee_link', '/fixed_finger_tip', rospy.Time(0))
        ee2fingertip=m3d.Transform()
        ee2fingertip.pos=tuple(ee2fingertip_temp[0])
        ee2fingertip_ori_temp = tf.transformations.euler_from_quaternion(tuple(ee2fingertip_temp[1]), axes='sxyz')
        ee2fingertip.orient.rotate_xb(ee2fingertip_ori_temp[0])
        ee2fingertip.orient.rotate_yb(ee2fingertip_ori_temp[1])
        ee2fingertip.orient.rotate_zb(ee2fingertip_ori_temp[2])
        print 'ee2fingertip', ee2fingertip
        if ee2fingertip.pos.y>0.02:
            break
    listener.waitForTransform('/ee_link', '/wrist_3_link', rospy.Time(), rospy.Duration(1.0))
    ee2wrist3_temp = listener.lookupTransform('/ee_link', '/wrist_3_link', rospy.Time(0))
    ee2wrist3=m3d.Transform()
    ee2wrist3.pos=tuple(ee2wrist3_temp[0])
    ee2wrist3_ori_temp = tf.transformations.euler_from_quaternion(tuple(ee2wrist3_temp[1]), axes='sxyz')
    ee2wrist3.orient.rotate_xb(ee2wrist3_ori_temp[0])
    ee2wrist3.orient.rotate_yb(ee2wrist3_ori_temp[1])
    ee2wrist3.orient.rotate_zb(ee2wrist3_ori_temp[2])
    Feasible_solution_exist=False
    for Go_index in range(len(pose_Go_stone_set)):
        for ori_index in range(8):
            clean_scene()  
            #rospy.sleep(0.5)
            group_gripper.go([-0.01567*(10)+0.62683,0,0,-0.03,0,0,0,0]) 
            rospy.sleep(0.5)
            target_fingertip_pos=m3d.Transform()
            fingertip_position = [pose_Go_stone_set[Go_index][0]+0.015*sin(ori_index*45*pi/180), pose_Go_stone_set[Go_index][1]-0.015*cos(ori_index*45*pi/180)]
            gripper_orientation = ori_index*45
            target_fingertip_pos.pos = (pose_Go_stone_set[Go_index][0]+0.015*sin(ori_index*45*pi/180), pose_Go_stone_set[Go_index][1]-0.015*cos(ori_index*45*pi/180), 0.16)
            target_fingertip_pos_temp = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(tf.transformations.quaternion_from_euler(pi,0,90*pi/180, axes='sxyz'), tf.transformations.quaternion_from_euler(0,30*pi/180,-ori_index*45*pi/180, axes='sxyz')), axes='sxyz')
            target_fingertip_pos.orient.rotate_xb(target_fingertip_pos_temp[0])
            target_fingertip_pos.orient.rotate_yb(target_fingertip_pos_temp[1])
            target_fingertip_pos.orient.rotate_zb(target_fingertip_pos_temp[2])
            base2ee = target_fingertip_pos*ee2fingertip.inverse
            baselink2base=m3d.Transform()
            baselink2base.orient.rotate_zb(pi)
            baselink2ee = baselink2base*target_fingertip_pos*ee2fingertip.inverse
            baselink2wrist3 = baselink2ee*ee2wrist3
            baselink2wrist3_orientation_matrix=np.zeros([4,4])
            baselink2wrist3_orientation_matrix[3,3]=1
            baselink2wrist3_orientation_matrix[:3,:3] = np.array(list(baselink2wrist3.orient)).transpose()
            pose_target = group_manipulator.get_current_pose().pose
            pose_target.position.x = baselink2wrist3.pos.x
            pose_target.position.y = baselink2wrist3.pos.y
            pose_target.position.z = baselink2wrist3.pos.z
            pose_target_orientation_temp = tf.transformations.quaternion_from_matrix(baselink2wrist3_orientation_matrix)
            pose_target.orientation.x = pose_target_orientation_temp[0]
            pose_target.orientation.y = pose_target_orientation_temp[1]
            pose_target.orientation.z = pose_target_orientation_temp[2]
            pose_target.orientation.w = pose_target_orientation_temp[3]
            group_manipulator.set_pose_target(pose_target)
            plan1 = group_manipulator.plan()
            if len(plan1.joint_trajectory.points)==0:
                first_step_pose = None
                continue
            group_manipulator.execute(plan1)
            pose_target = group_manipulator.get_current_pose().pose
            pose_target.position.z -= 0.03
            group_manipulator.set_pose_target(pose_target)
            plan1 = group_manipulator.plan()  
            group_manipulator.execute(plan1)
            if len(plan1.joint_trajectory.points)==0:
                first_step_pose = None
                continue
            group_gripper.go([-0.01567*(10)+0.62683,0,-0.018,-0.03,0,0,0,0]) 
            rospy.sleep(0.5)
            add_environment()
            rospy.sleep(0.5)
            res=collision_checker_node.start_collision_checker()
            if res:
                first_step_pose = None
                continue
            else:
                print fingertip_position, gripper_orientation, 'Feasible solution exists'
                Feasible_solution_exist=True
                break
        if Feasible_solution_exist==True:
            break
            


    moveit_commander.roscpp_shutdown()

    print 'STOPPING'
    
process_start()
