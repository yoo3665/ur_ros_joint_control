#!/usr/bin/env python2

import rospy
import threading
import time

from pynput import keyboard

import actionlib
from ur_dashboard_msgs.msg import SetModeAction, \
                                    SetModeGoal, \
                                    RobotMode
from ur_dashboard_msgs.srv import GetRobotMode, \
                                    GetProgramState, \
                                    GetLoadedProgram, \
                                    GetSafetyMode, \
                                    Load
from controller_manager_msgs.srv import SwitchControllerRequest, \
                                        SwitchController
from std_srvs.srv import Trigger
import std_msgs.msg
from std_msgs.msg import Bool

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

ALL_CONTROLLERS = [
        "scaled_pos_joint_traj_controller",
        "pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "vel_joint_traj_controller",
        "joint_group_vel_controller",
        "forward_joint_traj_controller",
        "forward_cartesian_traj_controller",
        "twist_controller",
        "pose_based_cartesian_traj_controller",
        "joint_based_cartesian_traj_controller",
        ]

class UR():
    def __init__(self):
        print("UR Class is Initializing")

        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cnt = 0
        # ROS Service Initialization
        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)

        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        self.s_connectToDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        self.s_quitFromDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)

        timeout = rospy.Duration(30)

        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
            print("set mode action client is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
            print("controller switch service is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller switch service. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.trajectory_client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        try:
            self.trajectory_client.wait_for_server(timeout)
            print("follow trajectory action client is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)

        # Running up the Manipulator
        resp = self.s_connectToDashboardServer()

        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)
        self.set_robot_to_mode(RobotMode.RUNNING)
        rospy.sleep(10)

        self.s_loadProgram("/programs/ros.urp")
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        resp = self.s_playProgram()
        rospy.sleep(0.5)

        self.switch_on_controller("scaled_pos_joint_traj_controller")
        rospy.sleep(0.5)

        rospy.Subscriber('/scaled_pos_joint_traj_controller/state',JointTrajectoryControllerState, self.callback)

        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    def move(self, key):
        if key == keyboard.KeyCode(char='q'):
            #print("joint_1 + ")
            self.position[0] = self.position[0]+0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])
            
        elif key == keyboard.KeyCode(char='a'):
            #print("joint_1 - ")
            self.position[0] = self.position[0]-0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='w'):
            #print("joint_2 + ")
            self.position[1] = self.position[1]+0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='s'):
            #print("joint_2 - ")
            self.position[1] = self.position[1]-0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='e'):
            #print("joint_3 + ")
            self.position[2] = self.position[2]+0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='d'):
            #print("joint_3 - ")    
            self.position[2] = self.position[2]-0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='r'):
            #print("joint_4 + ")
            self.position[3] = self.position[3]+0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='f'):
            #print("joint_4 - ") 
            self.position[3] = self.position[3]+0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='t'):
            #print("joint_5 + ")
            self.position[4] = self.position[4]+0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='g'):
            #print("joint_5 - ")
            self.position[4] = self.position[4]-0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='y'):
            #print("joint_6 + ")
            self.position[5] = self.position[5]+0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])

        elif key == keyboard.KeyCode(char='h'):
            #print("joint_6 - ")     
            self.position[5] = self.position[5]-0.1
            self.joint_traj(self.position[0],
                            self.position[1],
                            self.position[2],
                            self.position[3],
                            self.position[4],
                            self.position[5])  

    def on_press(self, key):
        #print('Key %s pressed' % key)
        threading.Thread(target=self.move, args=(key,)).start()  # <- note extra ','
        
         

    def on_release(self, key):
        #print('Key %s released' %key)
        if key == keyboard.Key.esc:
            self.finalize()
            return False

    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode = target_mode
        goal.play_program = True # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = False

        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()
        return self.set_mode_client.get_result().success

    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        print(result)

    def joint_traj(self, _1, _2, _3, _4, _5, _6):
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        position = [_1, _2, _3, _4, _5, _6]
        point.positions = position
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)

        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()
        #rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def callback(self, data):
        self.position = list(data.actual.positions)

    def finalize(self):
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('ur_ros_joint_control_node') 

    try:
        ur = UR()
    except rospy.ROSInterruptException: pass