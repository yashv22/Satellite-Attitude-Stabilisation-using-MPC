# Project Information

**Project:** 3U CubeSat Attitude Stabilization using Model Predictive Control  
**Course:** AE-642 Term Project  
**Project Supervisor:** Dr. D.K. Giri  
**Authors:** Yash Verma and Adarsh Anurag  
**Timeline:** Aug 2025 – Nov 2025

## Methods
Quaternion kinematics; nonlinear rigid-body dynamics; magnetorquer control; LTV linearization; discrete-time state-space modeling; constrained MPC; quadratic programming; MATLAB `quadprog`; PID/PD baseline comparison.

## Source-grounded setup
- Circular LEO, 500 km altitude, 97.4° inclination
- Inertia: diag(0.01, 0.01, 0.005) kg·m²
- Initial angular velocity: [0.09, 0, 0.03] rad/s
- Magnetorquer limit: ±0.1 A·m²
- Simulation: 3 orbits (~3 × 5700 s)
- Supplied implementation: 10 s prediction step, horizon 8

## CV results
The CV reports 42% faster detumbling and 35% lower actuator saturation. The supplied PDF qualitatively reports faster settling and lower saturation, and includes code that computes those metrics, but does not provide a numerical table explicitly showing the 42% and 35% values. These percentages are therefore retained as reported project outcomes.
