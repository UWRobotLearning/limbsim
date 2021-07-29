"""Example of running A1 robot with position control.

To run:
python a1_robot_exercise.py
"""
import time

import numpy as np
import pybullet
import pybullet_data
from pybullet_utils import bullet_client
from quadsim.robots.a1 import A1
from quadsim.robots.motors import MotorCommand
from quadsim.simulator import SimulatorConf


def get_action(t):
    mid_action = np.array([0.0, 0.9, -1.8] * 4)
    amplitude = np.array([0.0, 0.2, -0.4] * 4)
    freq = 1.0
    return MotorCommand(
        desired_position=mid_action + amplitude * np.sin(2 * np.pi * freq * t)
    )


def main():
    sim_conf = SimulatorConf(connection_mode=pybullet.GUI, on_rack=False,)

    p = bullet_client.BulletClient(connection_mode=sim_conf.connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0.0, 0.0, -9.8)

    robot = A1(pybullet_client=p, sim_conf=sim_conf)
    robot.reset()

    for _ in range(10000):
        action = get_action(robot.time_since_reset)
        robot.step(action)
        time.sleep(0.002)
        print(robot.base_orientation_rpy)


if __name__ == "__main__":
    main()
