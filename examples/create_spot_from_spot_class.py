"""Instantiate a Robot object configured as Spot.
1. Set simulator configuration and instantiate pybullet client instance
2. Instantiate Spot with sane defualts provided by Spot class.

To run:
python examples/create_spot_from_spot_class.py
"""
import pybullet
import pybullet_data
from limbsim.robots.spot import Spot
from limbsim.simulator import SimulatorConf
from pybullet_utils import bullet_client

if __name__ == "__main__":

    # Configure Simulator
    sim_conf = SimulatorConf(
        connection_mode=pybullet.DIRECT,
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

    # Create Spot Robot
    robot = Spot(pybullet_client=p, sim_conf=sim_conf,)

    # Print out all currently set instance attributes
    for attr, val in robot.__dict__.items():
        print(attr, "=", val)
        if attr == "_motor_group":
            print("======MotorGroup:")
            for bttr, vbl in val.__dict__.items():
                print(bttr, "=", vbl)
