"""Instantiate a Robot object configured as an A1.
1. Set simulator configuration and instantiate pybullet client instance
2. Instantiate A1 with sane defualts provided by A1 class.

To run:
python examples/create_a1_from_a1_class.py
"""
import pybullet
import pybullet_data
from limbsim.robots.a1 import A1
from limbsim.simulator import SimulatorConf
from pybullet_utils import bullet_client

if __name__ == "__main__":

    # Configure Simulator
    sim_conf = SimulatorConf(
        connection_mode=pybullet.GUI,
        timestep=0.002,
        action_repeat=1,
        reset_time=3,
        num_solver_iterations=30,
        init_position=(0.0, 0.0, 0.32),
        init_rack_position=(0.0, 0.0, 1),
        on_rack=False,
    )

    # Set up pybullet client
    p = bullet_client.BulletClient(connection_mode=sim_conf.connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    robot = A1(pybullet_client=p, sim_conf=sim_conf,)

    # Print out all currently set instance attributes
    for attr, val in robot.__dict__.items():
        print(attr, "=", val)
        if attr == "_motor_group":
            print("======MotorGroup:")
            for bttr, vbl in val.__dict__.items():
                print(bttr, "=", vbl)
