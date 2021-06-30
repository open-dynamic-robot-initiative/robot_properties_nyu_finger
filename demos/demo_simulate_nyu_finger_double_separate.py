#!/usr/bin/env python

"""demo_simulate_nyu_finger_double

Simple demo showing how the simulation setup works.

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import time
from robot_properties_nyu_finger.wrapper import NYUFingerRobot
from bullet_utils.env import BulletEnv
import numpy as np

if __name__ == "__main__":

    # ! Create a Pybullet simulation environment before any robots !
    env = BulletEnv()

    # Create a robot instance. This adds the robot to the simulator as well.
    finger0 = NYUFingerRobot(finger_index=0)
    finger1 = NYUFingerRobot(finger_index=1)

    # Add the robot to the env to update the internal structure of the robot
    # at every simulation steps.
    env.add_robot(finger0)
    env.add_robot(finger1)

    # Some control.
    tau0 = np.zeros(finger0.nv)
    tau1 = np.zeros(finger1.nv)

    # Reset the robot to some initial state.
    q = 0.5 * np.ones(finger0.nq)
    dq = np.zeros(finger0.nv)
    finger0.reset_state(q + 0.05 * np.random.standard_normal(finger0.nq), dq)
    finger1.reset_state(q + 0.05 * np.random.standard_normal(finger1.nq), dq)

    # Run the simulator for 2000 steps
    for i in range(2000):
    # TODO: Implement a controller here.
        finger0.send_joint_command(tau0)
        finger1.send_joint_command(tau1)

    # Step the simulator.
        env.step(
            sleep=True
        )  # You can sleep here if you want to slow down the replay

    # Read the final state and forces after the stepping.
    q, dq = finger0.get_state()
    active_eff, forces = finger0.get_force()
    print("finger 0: ")
    print("q", q)
    print("dq", dq)
    print("active eff", active_eff)
    print("forces", forces)
    print("\n")

    q, dq = finger1.get_state()
    active_eff, forces = finger1.get_force()
    print("finger 0: ")
    print("q", q)
    print("dq", dq)
    print("active eff", active_eff)
    print("forces", forces)
