import pybullet
import pybullet_data
from pybullet_utils import bullet_client
from limbsim.robots.motors import MotorControlMode
from limbsim.robots.motors import MotorGroup
from limbsim.robots.motors import MotorModel
from limbsim.robots.robot import Robot
from limbsim.simulator import SimulatorConf

if __name__ == "__main__":

    sim_conf = SimulatorConf(connection_mode=pybullet.GUI, on_rack=True,)
    sim_conf.init_position = list(sim_conf.init_position)

    # Set up pybullet client
    p = bullet_client.BulletClient(connection_mode=sim_conf.connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    motor1 = MotorModel(
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

    motor2 = MotorModel(
        name="FR_upper_joint",
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

    motor_group = MotorGroup(motors=[motor1, motor2])

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
    """
    TODO: Add print functionality to Robot
    BODY: Calling `print(robot)` or `pprint(robot)` iterates through
    BODY: the current robot configuration.
    """
