import pybullet
import pybullet_data
from pybullet_utils import bullet_client
from quadsim.robots.motors import MotorControlMode
from quadsim.robots.motors import MotorGroup
from quadsim.robots.motors import MotorModel
from quadsim.robots.robot import Robot
from quadsim.simulator import SimulatorConf

if __name__ == "__main__":

    sim_conf = SimulatorConf(show_gui=False, on_rack=True,)
    sim_conf.init_position = list(sim_conf.init_position)

    # Set up pybullet client
    if sim_conf.show_gui:
        p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
    else:
        p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
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
