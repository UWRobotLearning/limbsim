"""Example of running Spot robot with position control.
Note: This is an estimated simulation of Spot, rather than an
accurate representation of the onboard controllers Spot posesses.

To run:
python spot_robot_exercise.py
"""
import time

import numpy as np
import pybullet
import pybullet_data
from limbsim.robots.motors import MotorCommand
from limbsim.robots.spot import Spot
from limbsim.simulator import SimulatorConf
from pybullet_utils import bullet_client


def get_action(t):
    mid_action = np.array([0.0, 0.9, -1.8] * 4)
    amplitude = np.array([0.0, 0.2, -0.4] * 4)
    freq = 1.0
    import ipdb; ipdb.set_trace()
    return MotorCommand(
        desired_position=mid_action + amplitude * np.sin(2 * np.pi * freq * t)
    )


def main():
    sim_conf = SimulatorConf(connection_mode=pybullet.GUI, on_rack=False,)

    p = bullet_client.BulletClient(connection_mode=sim_conf.connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0.0, 0.0, -9.8)

    robot = Spot(pybullet_client=p, sim_conf=sim_conf)
    robot.reset()

    for _ in range(10000):
        action = get_action(robot.time_since_reset)
        robot.step(action)
        time.sleep(0.002)
        print(robot.base_orientation_rpy)


if __name__ == "__main__":
    main()
