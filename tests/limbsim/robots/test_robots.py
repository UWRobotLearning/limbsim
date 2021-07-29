"""Unit tests for the motor model class."""
import numpy as np
import pybullet
import pybullet_data
import pytest
from limbsim.robots.a1 import A1
from limbsim.robots.motors import MotorCommand
from limbsim.simulator import SimulatorConf
from pybullet_utils import bullet_client


def setup():

    # Configure Simulator
    sim_conf = SimulatorConf(
        connection_mode=pybullet.DIRECT,
        timestep=0.002,
        action_repeat=1,
        reset_time=3,
        num_solver_iterations=30,
        init_position=(0.0, 0.0, 0.32),
        on_rack=False,
        init_rack_position=(0.0, 0.0, 1),
    )

    # Create pybullet client
    p = bullet_client.BulletClient(connection_mode=sim_conf.connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.resetSimulation()
    return p, sim_conf


@pytest.mark.parametrize(
    "on_rack, hard_reset",
    [
        pytest.param(True, True, id="0",),
        pytest.param(True, False, id="1",),
        pytest.param(False, True, id="2",),
        pytest.param(False, False, id="3",),
    ],
)
def test_reset(on_rack, hard_reset):
    pybullet_client, sim_conf = setup()
    sim_conf.on_rack = on_rack

    robot = A1(pybullet_client=pybullet_client, sim_conf=sim_conf)
    if hard_reset:
        robot._pybullet_client.resetSimulation()
        robot._pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        robot._pybullet_client.loadURDF("plane.urdf")
    robot.reset(hard_reset=hard_reset)
    if on_rack:
        np.testing.assert_allclose(
            robot.base_position, sim_conf.init_rack_position, atol=0.05
        )
    else:
        np.testing.assert_allclose(
            robot.base_position, sim_conf.init_position, atol=0.1
        )
    np.testing.assert_allclose(robot.base_orientation_rpy, [0.0, 0.0, 0.0], atol=0.03)
    np.testing.assert_allclose(
        robot.base_orientation_quat, [0.0, 0.0, 0.0, 1.0], atol=0.01
    )
    np.testing.assert_allclose(robot.base_velocity, [0.0, 0.0, 0.0], atol=0.01)
    np.testing.assert_allclose(robot.base_rpy_rate, [0.0, 0.0, 0.0], atol=0.01)
    if on_rack:
        np.testing.assert_allclose(
            robot.motor_angles, robot._motor_group._init_positions, atol=0.01
        )
    else:
        np.testing.assert_allclose(
            robot.motor_angles, robot._motor_group._init_positions, atol=0.1
        )
    np.testing.assert_allclose(robot.motor_velocities, np.zeros(12), atol=0.01)
    np.testing.assert_allclose(
        robot.control_timestep, sim_conf.timestep * sim_conf.action_repeat,
    )
    np.testing.assert_allclose(robot.time_since_reset, 0.0)


def test_step():
    pybullet_client, sim_conf = setup()
    robot = A1(pybullet_client, sim_conf=sim_conf)
    motor_command = MotorCommand(desired_position=robot._motor_group.init_positions)
    for _ in range(10):
        robot.step(motor_command)
    np.testing.assert_equal(robot.time_since_reset, 10 * robot.control_timestep)
