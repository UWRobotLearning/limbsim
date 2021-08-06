"""Instantiate a Robot object configured as an A1.
1. Set simulator configuration and instantiate pybullet client instance
2. Instantiate each individual motor for A1 morphology
3. Instantiate a MotorGroup containing all motors
4. Instantiate a Robot using MotorGroup and A1 urdf.

To run:
python examples/create_a1_from_robot_class.py
"""
import pybullet
import pybullet_data
from pybullet_utils import bullet_client
from limbsim.robots.motors import MotorControlMode
from limbsim.robots.motors import MotorGroup
from limbsim.robots.motors import MotorModel
from limbsim.robots.robot import Robot
from limbsim.simulator import SimulatorConf

if __name__ == "__main__":

    # Configure Simulator
    sim_conf = SimulatorConf(
        connection_mode=pybullet.GUI,
        timestep=0.002,
        action_repeat=1,
        reset_time=3,
        num_solver_iterations=30,
        init_position=(0.0, 0.0, 0.32),
        on_rack=False,
        init_rack_position=(0.0, 0.0, 1),
    )

    # Set up pybullet client
    p = bullet_client.BulletClient(connection_mode=sim_conf.connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Create all motors manually
    motor_FR_hip = MotorModel(
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
    )

    motor_FR_upper = MotorModel(
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
    )

    motor_FR_lower = MotorModel(
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
    )

    motor_FL_hip = MotorModel(
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
    )

    motor_FL_upper = MotorModel(
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
    )

    motor_FL_lower = MotorModel(
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
    )

    motor_RR_hip = MotorModel(
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
    )

    motor_RR_upper = MotorModel(
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
    )

    motor_RR_lower = MotorModel(
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
    )

    motor_RL_hip = MotorModel(
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
    )

    motor_RL_upper = MotorModel(
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
    )

    motor_RL_lower = MotorModel(
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
    )

    motor_group = MotorGroup(
        motors=[
            motor_FL_hip,
            motor_FL_upper,
            motor_FL_lower,
            motor_FR_hip,
            motor_FR_upper,
            motor_FR_lower,
            motor_RR_hip,
            motor_RR_upper,
            motor_RR_lower,
            motor_RL_hip,
            motor_RL_upper,
            motor_RL_lower,
        ]
    )

    robot = Robot(
        pybullet_client=p,
        sim_conf=sim_conf,
        urdf_path="a1/a1.urdf",
        motors=motor_group,
        base_joint_names=[],
        foot_joint_names=[
            "FR_toe_fixed",
            "FL_toe_fixed",
            "RR_toe_fixed",
            "RL_toe_fixed",
        ],
    )

    # Print out all currently set instance attributes
    for attr, val in robot.__dict__.items():
        print(attr, "=", val)
        if attr == "_motor_group":
            print("======MotorGroup:")
            for bttr, vbl in val.__dict__.items():
                print(bttr, "=", vbl)
