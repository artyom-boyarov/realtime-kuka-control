# KUKA iiwa PyBullet UDP simulation

Install Python dependencies with `python3 -m pip install -r requirements.txt`. Start the simulator and controller in separate terminals:

```sh
python3 simulation.py
python3 pd_client.py
```

Use `python3 simulation.py --headless` without a display. Both programs accept `--help`; the client exposes PD gains and a per-joint torque limit. Stop either program with Ctrl+C.

The simulator listens on `127.0.0.1:7200` for torque commands and sends joint angles to `127.0.0.1:7201` at 240 Hz by default. Each UDP datagram is exactly 28 bytes: seven little-endian IEEE 754 float32 values (`<7f` in Python), ordered as URDF joints `joint1` through `joint7`. Commands are in N·m and angles are in radians. The simulator uses the most recently received valid command; malformed and non-finite packets are ignored. Torques are reset to zero after 0.25 seconds without a valid command (configurable with `--timeout`). UDP has no delivery or timing guarantees.

The client binds port 7201, estimates velocity from successive angle samples, and sends `-kp * angle - kd * velocity`, clipped to its configured torque limit. The arm is fixed at its base, gravity is enabled, and PyBullet's default joint motors are disabled. The client does not compensate for gravity, so a nonzero steady-state error is possible. Gains are starting values for this model and may need tuning.
