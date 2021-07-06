"""config

Store the configuration of the Solo family robots.

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
from math import pi
from os.path import join, dirname
from os import environ
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper
from robot_properties_nyu_finger.utils import find_paths


class NYUFingerAbstract(object):
    """ Abstract class used for all NYUFinger robots. """

    # PID gains
    kp = 5.0
    kd = 0.1
    ki = 0.0

    # The Kt constant of the motor [Nm/A]: tau = I * Kt.
    motor_torque_constant = 0.025

    # Control time period.
    control_period = 0.001
    dt = control_period

    # MaxCurrent = 12 # Ampers
    max_current = 2

    # Maximum torques.
    max_torque = motor_torque_constant * max_current

    # Maximum control one can send, here the control is the current.
    max_control = max_current

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain = 1.0

    max_qref = pi
    
    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path
        )
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names

class NYUFingerConfig(NYUFingerAbstract):
    robot_family = "nyu_finger"
    robot_name = "nyu_finger"

    paths = find_paths(robot_name)
    meshes_path = paths["resources"]
    dgm_yaml_dir = paths["dgm_yaml_dir"]
    dgm_yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ration = 9.0

    # pinocchio model.
    pin_robot = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path
    )
    pin_robot.model.rotorInertia[6:] = motor_inertia
    pin_robot.model.rotorGearRatio[6:] = motor_gear_ration

    robot_model = pin_robot.model
    mass = np.sum([i.mass for i in robot_model.inertias])

    # End effectors informations
    end_effector_names = ['finger_lower_to_tip_joint']
    end_eff_ids = [robot_model.getFrameId('finger_lower_to_tip_joint')]
    nb_ee = len(end_effector_names)


    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6

    joint_names = ["finger_base_to_upper_joint", 
                   "finger_upper_to_middle_joint", 
                   "finger_middle_to_lower_joint"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(3))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0., 0., 0.]
    initial_velocity = [0., 0., 0.]

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)

class NYUFingerDoubleConfig0(NYUFingerConfig):
    robot_family = "nyu_finger"
    robot_name = "nyu_finger_double0"

    paths = find_paths(robot_name)
    meshes_path = paths["resources"]
    dgm_yaml_dir = paths["dgm_yaml_dir"]
    dgm_yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ration = 9.0

    # pinocchio model.
    pin_robot = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path
    )
    pin_robot.model.rotorInertia[6:] = motor_inertia
    pin_robot.model.rotorGearRatio[6:] = motor_gear_ration

    robot_model = pin_robot.model
    mass = np.sum([i.mass for i in robot_model.inertias])

    # End effectors informations
    end_effector_names = ['finger0_lower_to_tip_joint']
    end_eff_ids = [robot_model.getFrameId('finger0_lower_to_tip_joint')]
    nb_ee = len(end_effector_names)

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6

    joint_names = ["finger0_base_to_upper_joint", 
                   "finger0_upper_to_middle_joint", 
                   "finger0_middle_to_lower_joint"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(3))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0., 0., 0.]
    initial_velocity = [0., 0., 0.]

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)

class NYUFingerDoubleConfig1(NYUFingerConfig):
    robot_family = "nyu_finger"
    robot_name = "nyu_finger1"

    paths = find_paths(robot_name)
    meshes_path = paths["resources"]
    dgm_yaml_dir = paths["dgm_yaml_dir"]
    dgm_yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ration = 9.0

    # pinocchio model.
    pin_robot = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path
    )
    pin_robot.model.rotorInertia[6:] = motor_inertia
    pin_robot.model.rotorGearRatio[6:] = motor_gear_ration

    robot_model = pin_robot.model
    mass = np.sum([i.mass for i in robot_model.inertias])

    # End effectors informations
    end_effector_names = ['finger1_lower_to_tip_joint']
    end_eff_ids = [robot_model.getFrameId('finger1_lower_to_tip_joint')]
    nb_ee = len(end_effector_names)

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6

    joint_names = ["finger1_base_to_upper_joint", 
                   "finger1_upper_to_middle_joint", 
                   "finger1_middle_to_lower_joint"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(3))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0., 0., 0.]
    initial_velocity = [0., 0., 0.]

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)


class NYUFingerDoubleConfig(NYUFingerAbstract):
    robot_family = "nyu_finger"
    robot_name = "nyu_finger_double"

    paths = find_paths(robot_name)
    meshes_path = paths["resources"]
    dgm_yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ration = 9.0

    # pinocchio model.
    pin_robot_wrapper = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path
    )
    pin_robot_wrapper.model.rotorInertia[6:] = motor_inertia
    pin_robot_wrapper.model.rotorGearRatio[6:] = motor_gear_ration
    pin_robot = pin_robot_wrapper

    robot_model = pin_robot_wrapper.model
    mass = np.sum([i.mass for i in robot_model.inertias])

    # End effectors informations
    end_effector_names = ['finger0_lower_to_tip_joint', 
                          'finger1_lower_to_tip_joint']
    end_eff_ids = [robot_model.getFrameId('finger0_lower_to_tip_joint'),
                   robot_model.getFrameId('finger1_lower_to_tip_joint')]
    nb_ee = len(end_effector_names)


    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6

    joint_names = ["finger0_base_to_upper_joint", 
                   "finger0_upper_to_middle_joint", 
                   "finger0_middle_to_lower_joint",
                   "finger1_base_to_upper_joint", 
                   "finger1_upper_to_middle_joint", 
                   "finger1_middle_to_lower_joint"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(3))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0., 0., 0., 0., 0., 0.]
    initial_velocity = [0., 0., 0., 0., 0., 0.]

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)
