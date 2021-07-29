"""Base class for all robots."""
from typing import Any
from typing import Tuple

from limbsim.robots.motors import MotorControlMode
from limbsim.robots.motors import MotorGroup
from limbsim.robots.motors import MotorModel
from limbsim.robots.robot import Robot
from limbsim.simulator import SimulatorConf


class A1(Robot):
    """A1 Robot."""

    def __init__(
        self,
        pybullet_client: Any = None,
        sim_conf: SimulatorConf = None,
        urdf_path: str = "a1/a1.urdf",
        motors: Tuple[MotorGroup, ...] = None,
        base_joint_names: Tuple[str, ...] = (),
        foot_joint_names: Tuple[str, ...] = (
            "FR_toe_fixed",
            "FL_toe_fixed",
            "RR_toe_fixed",
            "RL_toe_fixed",
        ),
    ) -> None:
        """Constructs an A1 robot and resets it to the initial states.
        Initializes a tuple with a single MotorGroup containing 12 MotoroModels.
        Each MotorModel is by default configured for the parameters of the A1.
        """
        if motors is None:
            motors = MotorGroup(
                (
                    MotorModel(
                        name="FR_hip_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.0,
                        min_position=-0.802851455917,
                        max_position=0.802851455917,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=1,
                    ),
                    MotorModel(
                        name="FR_upper_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.9,
                        min_position=-1.0471975512,
                        max_position=4.18879020479,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                    MotorModel(
                        name="FR_lower_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=-1.8,
                        min_position=-2.6965336943,
                        max_position=-0.916297857297,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                    MotorModel(
                        name="FL_hip_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.0,
                        min_position=-0.802851455917,
                        max_position=0.802851455917,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=1,
                    ),
                    MotorModel(
                        name="FL_upper_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.9,
                        min_position=-1.0471975512,
                        max_position=4.18879020479,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                    MotorModel(
                        name="FL_lower_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=-1.8,
                        min_position=-1.0471975512,
                        max_position=4.18879020479,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                    MotorModel(
                        name="RR_hip_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.0,
                        min_position=-0.802851455917,
                        max_position=0.802851455917,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=1,
                    ),
                    MotorModel(
                        name="RR_upper_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.9,
                        min_position=-1.0471975512,
                        max_position=4.18879020479,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                    MotorModel(
                        name="RR_lower_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=-1.8,
                        min_position=-2.6965336943,
                        max_position=-0.916297857297,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                    MotorModel(
                        name="RL_hip_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.0,
                        min_position=-0.802851455917,
                        max_position=0.802851455917,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=1,
                    ),
                    MotorModel(
                        name="RL_upper_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=0.9,
                        min_position=-1.0471975512,
                        max_position=4.18879020479,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                    MotorModel(
                        name="RL_lower_joint",
                        motor_control_mode=MotorControlMode.POSITION,
                        init_position=-1.8,
                        min_position=-2.6965336943,
                        max_position=-0.916297857297,
                        min_velocity=-16,
                        max_velocity=16,
                        min_torque=-33.5,
                        max_torque=33.5,
                        kp=100,
                        kd=2,
                    ),
                )
            )
        super().__init__(
            pybullet_client,
            sim_conf,
            urdf_path,
            motors,
            base_joint_names,
            foot_joint_names,
        )
