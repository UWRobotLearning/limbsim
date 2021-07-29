from quadsim.robots.motors import MotorControlMode
from quadsim.robots.motors import MotorGroup
from quadsim.robots.motors import MotorModel

if __name__ == "__main__":
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
    # Print out all currently set instance attributes
    for attr, val in motor_group.__dict__.items():
        print(attr, "=", val)
        if attr == "_motors":
            for motor_num, motor in enumerate(val):
                print(f"===Motor {motor_num+1}:")
                for bttr, vbl in motor.__dict__.items():
                    print(bttr, "=", vbl)
            print("===MotorGroup:")
    """
    TODO: Add print functionality to MotorGroup
    BODY: Calling `print(motor_group)` or `pprint(motor_group)` iterates through
    BODY: the motors in `motor_group`, displaying each of their parameters.
    """
