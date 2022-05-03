#! /usr/bin/env python3

import typing as tp
from math import radians, degrees
import rospy

# from rospy.parameter import Parameter

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import JointState

from copy import copy

from transmission import Transmission, Kinematic
from motor import Motor, get_bus
from hal_config import M_BASE, M_EPAULE, M_COUDE, M_POIGNET_1, M_POIGNET_2, M_POIGNET_3
from hal_config import T_BASE, T_EPAULE, T_COUDE, T_POIGNET_1, T_POIGNET_2, T_POIGNET_3
#from .homing_controller import HomingController, HomingState


JOINTS_NAMES = ['frontal_hip', 'sagittal_hip', 'knee']
class HardwareAbstractionLayer():

    def __init__(self, period : float = 0.01):
        self.is_calibrating = False
        self.controllers = []
        self.motors = []
        self.motors_target = []
        self.transmissions = []
        self.joints_targets = []

        self.init_drivers()
        self.init_transmissions()
        
        self.joint_publisher_ = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.motor_publisher_ = rospy.Publisher('/motor_states', JointState, queue_size=1)
        self.motor_target_publisher_ = rospy.Publisher('/motor_targets', JointState, queue_size=1)
        self.motor_target_subscriber_ = rospy.Subscriber('/joint_position_cmd', JointTrajectoryPoint, self.update_cmd,1)

        self.init_parameters()
        self.init_targets()

        self.period = period
        self.timer = rospy.Timer(rospy.Duration(self.period), self.routine)
        rospy.loginfo(f"Hardware Abstraction Layer is ready, period: {period} sec")
        
        for motor in self.motors:
            motor.start()

        # self.start_calibration()

    # def start_calibration(self):
    #     self.is_calibrating = True
    #     for motor, transmission in zip(self.motors, self.transmissions):
    #         pos, _ = motor.state()
    #         controller = HomingController(motor.name, pos, transmission.high * transmission.ratio)
    #         self.controllers.append(controller)
    #     self.controllers[0].start()

# ========== Initialisation ==========

    def init_drivers(self):
        bus = get_bus()
        self.motors.append(Motor(M_BASE, bus))
        self.motors.append(Motor(M_EPAULE, bus))
        #self.motors.append(Motor(M_COUDE, bus))
        #self.motors.append(Motor(M_POIGNET_1, bus))
        #self.motors.append(Motor(M_POIGNET_2, bus))
        #self.motors.append(Motor(M_POIGNET_3, bus))
        rospy.loginfo(f'All motors initialized')

    def init_transmissions(self):
        transmission_conf = [T_BASE, T_EPAULE, T_COUDE, T_POIGNET_1, T_POIGNET_2, T_POIGNET_3]

        for i, motor in enumerate(self.motors):
            conf = copy(transmission_conf[i])
            # Read initial position and use it as the transmission zero.
            conf.zero = motor.position()
            self.transmissions.append(Transmission(conf))
        rospy.loginfo(f'All transmissions initialized')

    def init_parameters(self):
        """
        init ros parameters
        """
        if rospy.has_param('/my_gains'):
            rospy.delete_param('/my_gains')

        for i, mot in enumerate(self.motors):
            rospy.set_param(f'/my_gains/position_gain_{mot.name}', mot.position_gain)
            rospy.set_param(f'/my_gains/velocity_gain_{mot.name}', mot.velocity_gain)
            rospy.set_param(f'/my_gains/integrator_gain_{mot.name}', mot.integrator_gain)  
        self.update_parameters(rospy.get_param('/my_gains'))

    def update_parameters(self, params : dict):
        for i,motor in enumerate(self.motors):
            for key in params:
                if key == f'position_gain_{motor.name}':
                    pos_gain = params[key]
                if key== f'velocity_gain_{motor.name}':
                    vel_gain = params[key]
                if key == f'integrator_gain_{motor.name}':
                    integrator_gain = params[key]
            motor.set_gains(pos_gain, vel_gain, integrator_gain)

    def init_targets(self) -> None:
        """
        @brief Initialize target joint position to initial position (stay in place)
        """
        self.motors_target = []
        for motor in self.motors:
            pos, vel = motor.state()
            motor_state = Kinematic(pos, vel)
            self.motors_target.append(motor_state)
        self.joints_targets = self.actuator_to_joint(self.motors_target)

# ========== transformation ==========

    def actuator_to_joint(self, motor_positions: tp.List[Kinematic]) -> tp.List[Kinematic]:
        joint_states = []
        for motor_state, transmission in zip(motor_positions, self.transmissions):
            joint_states.append(transmission.actuator_to_joint(motor_state))
        
        return joint_states

    def joint_to_actuator(self, joint_targets: tp.List[Kinematic]) -> tp.List[float]:
        motor_targets = []
        for transmission, target in zip(self.transmissions, joint_targets):
            motor_targets.append(transmission.joint_to_actuator(target))
        return motor_targets

# ========== Read write ==========

    def update_cmd(self, msg : JointTrajectoryPoint, test):
        for i in range(len(msg.positions)):
            self.joints_targets[i] = Kinematic(msg.positions[i], msg.velocities[i])

    def send_targets(self, motor_targets: tp.List[Kinematic]):
        motor_msg = JointState()
        motor_msg.header.stamp = rospy.get_rostime()

        for i, target in enumerate(motor_targets):
            motor_msg.name.append(self.motors[i].name)
            motor_msg.position.append(target.position)
            motor_msg.velocity.append(target.velocity)
            self.motors[i].set_kinematic_target(target.position, target.velocity)
        self.motor_target_publisher_.publish(motor_msg)

    def read_state(self) -> tp.List[Kinematic]:
        motors_state = []
        motor_msg = JointState()
        motor_msg.header.stamp = rospy.get_rostime()
        for motor in self.motors:
            pos, vel = motor.state()
            motor_state = Kinematic(pos, vel)
            motors_state.append(motor_state)
            motor_msg.name.append(motor.name)
            motor_msg.position.append(motor_state.position)
            motor_msg.velocity.append(motor_state.velocity)
        self.motor_publisher_.publish(motor_msg)
        return motors_state

    def publish_joints_state(self, joints_state: tp.List[Kinematic]):
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.get_rostime()
        for i, joint in enumerate(joints_state):
            joint_msg.name.append(self.motors[i].name)
            joint_msg.position.append(joint.position)
            joint_msg.velocity.append(joint.velocity)
        self.joint_publisher_.publish(joint_msg)

# ========== Fonctionnement ==========

    def routine(self, timer):
        self.motors_state = self.read_state()
        self.joint_states = self.actuator_to_joint(self.motors_state)
        self.publish_joints_state(self.joint_states)

        self.motors_target = self.joint_to_actuator(self.joints_targets)
        
        self.send_targets(self.motors_target)

    def stop(self):
        rospy.loginfo(f'Stopping HAL Node')
        for motor in self.motors:
            motor.tm.idle()



def main(args=None):
    rospy.init_node('hal',anonymous=True)
    try:
        hal_object = HardwareAbstractionLayer(0.01)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    hal_object.stop()


if __name__ == '__main__':
    main()