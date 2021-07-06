"""nyufingerwrapper

NYU Finger pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import pybullet
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_nyu_finger.config import NYUFingerConfig, NYUFingerDoubleConfig

dt = 1e-3


class NYUFingerRobot(PinBulletWrapper):
    def __init__(
        self,
        pos=None,
        orn=None,
        useFixedBase=True,
        config=None
    ):

        # Load the robot
        if pos is None:
            pos = [0.0, 0.0, 0.0]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        if config is None:
            config = NYUFingerConfig()

        pybullet.setAdditionalSearchPath(config.meshes_path)
        self.urdf_path = config.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos,
            orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )

        # Create the robot wrapper in pinocchio.
        self.pin_robot = config.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.end_effector_names = config.end_effector_names
        self.end_eff_ids = config.end_eff_ids
        self.joint_names = config.joint_names
        self.nb_ee = len(self.end_effector_names)

        # Creates the wrapper by calling the super.__init__.
        super().__init__(
            self.robotId,
            self.pin_robot,
            self.joint_names,
            self.end_effector_names,
            useFixedBase,
        )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)


class NYUFingerDoubleRobot(PinBulletWrapper):
    def __init__(
        self,
        pos=None,
        orn=None,
        useFixedBase=True
    ):

        # Load the robot
        if pos is None:
            pos = [0.0, 0.0, 0.0]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(NYUFingerDoubleConfig.meshes_path)
        self.urdf_path = NYUFingerDoubleConfig.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos,
            orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )

        # Create the robot wrapper in pinocchio.
        self.pin_robot = NYUFingerDoubleConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.end_effector_names = NYUFingerDoubleConfig.end_effector_names
        self.end_eff_ids = NYUFingerDoubleConfig.end_eff_ids
        self.joint_names = NYUFingerDoubleConfig.joint_names
        self.nb_ee = len(self.end_effector_names)

        # Creates the wrapper by calling the super.__init__.
        super().__init__(
            self.robotId,
            self.pin_robot,
            self.joint_names,
            self.end_effector_names,
            useFixedBase,
        )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)
