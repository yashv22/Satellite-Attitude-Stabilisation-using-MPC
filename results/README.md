# Results

The report compares PID/PD and MPC over a three-orbit simulation.

- MPC shows faster angular-velocity reduction and smoother convergence.
- MPC reduces quaternion-vector attitude error more rapidly.
- MPC respects the ±0.1 A·m² magnetorquer limits while reducing unnecessary saturation.
- The report states that MPC achieves faster settling and lower actuator saturation fraction.

The four figures in `../assets/` are extracted directly from pages 7–10 of the supplied report.

The MATLAB implementation computes settling time, peak overshoot, torque RMS, and saturation fraction. Exact numerical values for the CV-reported 42% and 35% improvements are not tabulated in the supplied PDF.
