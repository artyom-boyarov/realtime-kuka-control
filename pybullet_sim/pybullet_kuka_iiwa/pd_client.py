#!/usr/bin/env python3
"""Control all iiwa joints toward zero using UDP angle feedback."""
import argparse
import math
import socket
import struct
import time

PACKET = struct.Struct('<7f')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kp', type=float, default=100.0, help='proportional gain (N m/rad)')
    parser.add_argument('--kd', type=float, default=10.0, help='derivative gain (N m s/rad)')
    parser.add_argument('--max-torque', type=float, default=100.0, help='per-joint torque limit (N m)')
    args = parser.parse_args()
    if (not all(math.isfinite(x) for x in (args.kp, args.kd, args.max_torque))
            or args.kp < 0 or args.kd < 0 or args.max_torque <= 0):
        parser.error('gains must be finite and nonnegative, and max-torque positive')
    feedback = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    feedback.bind(('127.0.0.1', 7201))
    feedback.settimeout(0.25)
    commands = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    previous = None
    previous_time = None
    try:
        while True:
            try:
                data, _ = feedback.recvfrom(65535)
            except socket.timeout:
                previous = previous_time = None
                continue
            if len(data) != PACKET.size:
                continue
            angles = PACKET.unpack(data)
            if not all(math.isfinite(x) for x in angles):
                continue
            now = time.monotonic()
            dt = now - previous_time if previous_time is not None else 0
            velocities = [(q - old) / dt for q, old in zip(angles, previous)] if dt > 0 else [0.0] * 7
            torques = [max(-args.max_torque, min(args.max_torque, -args.kp * q - args.kd * v))
                       for q, v in zip(angles, velocities)]
            commands.sendto(PACKET.pack(*torques), ('127.0.0.1', 7200))
            previous, previous_time = angles, now
    except KeyboardInterrupt:
        pass
    finally:
        commands.sendto(PACKET.pack(*([0.0] * 7)), ('127.0.0.1', 7200))
        feedback.close()
        commands.close()


if __name__ == '__main__':
    main()
