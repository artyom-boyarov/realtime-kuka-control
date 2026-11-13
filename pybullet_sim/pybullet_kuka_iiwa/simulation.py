#!/usr/bin/env python3
"""Run the iiwa simulation and exchange seven-float UDP datagrams."""
import argparse
import math
from pathlib import Path
import socket
import struct
import tempfile
import time
import numpy as np

import pybullet as p

PACKET = struct.Struct('<7f')
ROOT = Path(__file__).resolve().parent
PACKAGE = ROOT / 'lbr_iiwa7_r800-urdf-package'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='run without a GUI')
    parser.add_argument('--rate', type=float, default=240.0, help='simulation steps per second')
    parser.add_argument('--timeout', type=float, default=0.25, help='seconds before stale torque is cleared')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('rate and timeout must be finite and positive')

    command = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command.bind(('127.0.0.1', 7200))
    command.setblocking(False)
    state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    connection = p.connect(p.DIRECT if args.headless else p.GUI)
    if connection < 0:
        raise RuntimeError('Could not connect to PyBullet')
    try:
        # PyBullet does not resolve ROS package:// URLs. Keep the original URDF intact.
        urdf = (PACKAGE / 'urdf/lbr_iiwa7_r800.urdf').read_text()
        urdf = urdf.replace('package://lbr_iiwa7_r800/', PACKAGE.as_posix() + '/')
        with tempfile.TemporaryDirectory() as directory:
            resolved = Path(directory) / 'iiwa.urdf'
            resolved.write_text(urdf)
            robot = p.loadURDF(str(resolved), useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
        joints = [i for i in range(p.getNumJoints(robot))
                  if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
        if len(joints) != 7:
            raise RuntimeError(f'Expected seven revolute joints, found {len(joints)}')
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / args.rate)
        p.setJointMotorControlArray(robot, joints, p.VELOCITY_CONTROL, forces=[0] * 7)
        # Effort limits from the URDF, used to hold position before torque commands arrive.
        hold_forces = [p.getJointInfo(robot, joint)[10] for joint in joints]
        torque = (0.0,) * 7
        last_command = float('-inf')
        deadline = time.monotonic()
        while p.isConnected(connection):
            # Drain the socket and apply the newest valid command.
            while True:
                try:
                    data, _ = command.recvfrom(65535)
                except BlockingIOError:
                    break
                if len(data) == PACKET.size:
                    values = PACKET.unpack(data)
                    if all(math.isfinite(value) for value in values):
                        torque, last_command = values, time.monotonic()
                        print(f"Torque command: {', '.join(f'{value:.3f}' for value in torque)}")
            if time.monotonic() - last_command > args.timeout:
                # No fresh torque command: hold a safe zero pose instead of
                # free-falling under gravity, which would otherwise drive a
                # joint into its mechanical limit before control engages.
                p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL,
                                             targetPositions=[0.2] * 7, forces=hold_forces)
            else:
                print(f"Applying torque: {', '.join(f'{value:.3f}' for value in torque)}")
                p.setJointMotorControlArray(robot, joints, p.TORQUE_CONTROL, forces=torque)
            p.stepSimulation()
            angles = [joint[0] for joint in p.getJointStates(robot, joints)]
            print(f"Joint angles: {', '.join(f'{np.rad2deg(angle):.3f}' for angle in angles)}")
            state.sendto(PACKET.pack(*angles), ('127.0.0.1', 7201))
            deadline += 1 / args.rate
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect(connection)
        command.close()
        state.close()


if __name__ == '__main__':
    main()
