import argparse
import logging
import threading

from incar_networking.robot_interface import IncarRobotInterface
from unitree_g1_extension.robot_interface.robot_real import G1RobotReal
from unitree_g1_extension.robot_interface.robot_simulation import G1RobotSim
from unitree_g1_extension.robot_interface.run import IncarG1Bridge
from inspire_extension.robot_interface.run import InspireHand

from .hand import InspireGripper


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="INCAR teleoperation of one arm of the G1 and its Inspire hand")
    parser.add_argument(
        "--robot",
        choices=["sim", "real"],
        default="sim",
        help="sim: LeRobot's MuJoCo G1 (default); real: the G1 on the gantry, via g1_server.py on the robot",
    )
    parser.add_argument(
        "--arm",
        choices=["right", "left"],
        default="left",
        help="arm to teleoperate (default right); left follows the right controller, mirrored",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="sim only: don't open the MuJoCo window, publish the sim cameras over ZMQ instead (for g1_sim cameras)",
    )
    parser.add_argument(
        "--hand-port",
        default="/dev/ttyUSB0",
        help="serial port of the Inspire hand (default /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--hand-mode",
        choices=["gripper", "hand"],
        default="gripper",
        help="use inspire hand as gripper (binary input) or as full hand (6 joint angles)",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    
    if args.robot == "real":
        robot = G1RobotReal(arm=args.arm)
    else:
        robot = G1RobotSim(arm=args.arm, viewer=not args.no_viewer)
        
    robot.connect()
    bridge = IncarG1Bridge(robot)
    
    
    if args.hand_mode == "hand":
        hand = InspireHand(serial_port=args.hand_port)
        command_hooks = {
                        "right.commands.arm.ee.velocity": bridge.on_right_ee_velocity,
                        "right.commands.hand.inspire": hand.move_joints
                    }
        loop_callbacks = [
            hand.set_state_and_publish,
            bridge.on_loop
                    ]
    else:
        gripper = InspireGripper(port=args.hand_port, module_name=f"{args.arm}_hand")
        gripper.start()
        command_hooks = {
                        "right.commands.arm.ee.velocity": bridge.on_right_ee_velocity,
                        "right.commands.gripper.openclose": gripper.on_gripper
                    }
        loop_callbacks = [
                        gripper.on_loop,  
                        bridge.on_loop
                    ]

    interface = IncarRobotInterface(
            0.01,
            command_hooks=command_hooks,
            loop_callbacks=loop_callbacks
        )
    
    try:
        interface.start("127.0.0.1")
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
