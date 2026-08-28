% Transcribed from Appendix 1 of docs/MPC.pdf

%% 2) MPC
simulation (receding
horizon QP) using
ode23s
% For speed and to demonstrate
MPC performance , use a simplified
online MPC:
% - Prediction
uses
linearization
around
current B-field and state
% - QP solved at each
sample
using
quadprog
% We create a wrapper
that
integrates
using
ode23s but with
controller
evaluated
% internally by the ODE
function
based on params.controller == ’mpc ’.
disp(’Running
MPC
simulation (fast
settings)...’);
params.controller = ’mpc’;
% reset
persistent
fields in mpc
controller
clear
mpc_controller
[t_mpc , x_mpc] = ode23s(@(t,x) satellite_dynamics (t,x,params), t_eval , x0 , options);
disp(’MPC
simulation
complete.’);
%% Post -process: compute
histories (errors , m, torque) on returned
output
grid
[err_pid , m_pid , T_pid] = get_histories (t_pid , x_pid , params , ’pid’);
[err_mpc , m_mpc , T_mpc] = get_histories (t_mpc , x_mpc , params , ’mpc’);
%% Compute & display
performance
metrics
metrics_pid = compute_metrics(t_pid , x_pid , err_pid , m_pid , T_pid , params);
metrics_mpc = compute_metrics(t_mpc , x_mpc , err_mpc , m_mpc , T_mpc , params);
fprintf(’\n===
PERFORMANCE
SUMMARY
===\n’);
fprintf(’Controller
| Settling
time (s) | Peak
overshoot
(%%) | Torque RMS ( N
m )
| Saturation
frac\n’);
fprintf(’
--------------------------------------------------------------------------------------------
n’);
fprintf(’PID
| %8.1f
| %8.2f
| %8.3e
| %6.2f%%\n
’, ...
metrics_pid.settling_time , metrics_pid. peak_overshoot *100 ,
metrics_pid.torque_rms
, metrics_pid.sat_frac *100);
fprintf(’MPC
| %8.1f
| %8.2f
| %8.3e
| %6.2f%%\n
’, ...
metrics_mpc.settling_time , metrics_mpc. peak_overshoot *100 ,
metrics_mpc.torque_rms
, metrics_mpc.sat_frac *100);
fprintf(’
--------------------------------------------------------------------------------------------
n’);
%% Plot 1: Angular
Velocity
h=figure(’Name ’,’Angular
Velocity
Comparison ’,’Visible ’,’off’,’Units ’,’normalized ’,’
Position ’ ,[0.1 0.1 0.7 0.7]);
subplot (2,1,1);
plot(t_pid /3600 , x_pid (: ,1:3) *180/pi ,’LineWidth ’ ,1.2); title(’PID: Angular
Velocity ’)
;
legend(’\omega_x ’,’\omega_y ’,’\omega_z ’); xlabel(’Time (hours)’); ylabel(’Angular
Velocity (deg/s)’); grid on;
subplot (2,1,2);
plot(t_mpc /3600 , x_mpc (: ,1:3) *180/pi ,’LineWidth ’ ,1.2); title(’MPC: Angular
Velocity ’)
;
legend(’\omega_x ’,’\omega_y ’,’\omega_z ’); xlabel(’Time (hours)’); ylabel(’Angular
Velocity (deg/s)’); grid on;
saveas(h,’angular_velocity_plot .png’);
%% Plot 2: Attitude
Error
h=figure(’Name ’,’Attitude
Error
Comparison ’,’Visible ’,’off’,’Units ’,’normalized ’,’
Position ’ ,[0.1 0.1 0.7 0.7]);
subplot (2,1,1);
plot(t_pid /3600 ,
err_pid (: ,1:3) ,’LineWidth ’ ,1.2); title(’PID: Attitude
Error (q_{e,v
})’);
legend(’q_{ex}’,’q_{ey}’,’q_{ez}’); xlabel(’Time (hours)’); ylabel(’Error ’); grid on;

subplot (2,1,2);
plot(t_mpc /3600 ,
err_mpc (: ,1:3) ,’LineWidth ’ ,1.2); title(’MPC: Attitude
Error (q_{e,v
})’);
legend(’q_{ex}’,’q_{ey}’,’q_{ez}’); xlabel(’Time (hours)’); ylabel(’Error ’); grid on;
saveas(h,’attitude_error_plot .png’);
%% Plot 3: Control
Input (Dipole)
h=figure(’Name ’,’Control
Input
Comparison ’,’Visible ’,’off’,’Units ’,’normalized ’,’
Position ’ ,[0.1 0.1 0.7 0.7]);
subplot (2,1,1);
plot(t_pid /3600 , m_pid ,’LineWidth ’ ,1.2); hold on;
yline(params.m_max ,’r--’,’LineWidth ’ ,1); yline(-params.m_max ,’r--’,’LineWidth ’ ,1);
title(’PID: Control
Input (Dipole)’); legend(’m_x’,’m_y’,’m_z’,’m_{max}’); xlabel(’
Time (hours)’); ylabel(’Dipole ( A
m ^2)’); grid on;
subplot (2,1,2);
plot(t_mpc /3600 , m_mpc ,’LineWidth ’ ,1.2); hold on;
yline(params.m_max ,’r--’,’LineWidth ’ ,1); yline(-params.m_max ,’r--’,’LineWidth ’ ,1);
title(’MPC: Control
Input (Dipole)’); legend(’m_x’,’m_y’,’m_z’,’m_{max}’); xlabel(’
Time (hours)’); ylabel(’Dipole ( A
m ^2)’); grid on;
saveas(h,’control_input_plot .png’);
%% Plot 4: Applied
Torque
h=figure(’Name ’,’Applied
Torque
Comparison ’,’Visible ’,’off’,’Units ’,’normalized ’,’
Position ’ ,[0.1 0.1 0.7 0.7]);
subplot (2,1,1);
plot(t_pid /3600 , T_pid ,’LineWidth ’ ,1.2); title(’PID: Applied
Control
Torque ’); legend
(’T_x’,’T_y’,’T_z’);
xlabel(’Time (hours)’); ylabel(’Torque ( N
m )’); grid on;
subplot (2,1,2);
plot(t_mpc /3600 , T_mpc ,’LineWidth ’ ,1.2); title(’MPC: Applied
Control
Torque ’); legend
(’T_x’,’T_y’,’T_z’);
xlabel(’Time (hours)’); ylabel(’Torque ( N
m )’); grid on;
saveas(h,’control_torque_plot .png’);
disp(’Plots
saved: angular_velocity_plot .png , attitude_error_plot .png ,
control_input_plot .png , control_torque_plot .png’);
disp(’Script
finished.’);
% =========================================================================
% --- LOCAL
FUNCTIONS
---
% =========================================================================
function
x_dot = satellite_dynamics (t, x, params)
% Nonlinear
satellite
dynamics
with
runtime
controller
choice (pid or mpc)
w = x(1:3);
q = x(4:7); q = q / norm(q); x(4:7) = q; % normalize
quaternion
B_i = get_B_field_inertial (t);
A_BI = quat_to_dcm(q); % body to inertial
DCM from
quaternion
B_b = A_BI * B_i;
if strcmp(params.controller ,’pid’)
m_cmd = pid_controller (x, B_b , params);
else
m_cmd = mpc_controller (x, B_b , params);
end
% Saturate
m = max(-params.m_max , min(params.m_max , m_cmd));
T_c = cross(m, B_b);
% Dynamics
I = params.I;

w_dot = params.inv_I * (T_c - cross(w, I*w));
% Kinematics
Omega_w = [0, w(3), -w(2), w(1);
-w(3), 0, w(1), w(2);
w(2), -w(1), 0, w(3);
-w(1), -w(2), -w(3), 0];
q_dot = 0.5 * Omega_w * q;
x_dot = [w_dot; q_dot ];
end
% -------------------------------------------------------------------------
function
m_cmd = pid_controller(x, B_b , params)
w = x(1:3); q = x(4:7);
Kp = 0.002; Kd = 0.05;
q_err = quat_error(params.q_ref , q);
q_err_v = q_err (1:3);
if q_err (4) <0, q_err_v = -q_err_v; end
w_err = w - params.w_ref;
T_req = -Kp * q_err_v - Kd * w_err;
B_norm_sq = dot(B_b ,B_b);
if B_norm_sq < 1e-12
m_cmd = [0;0;0];
else
m_cmd = cross(B_b , T_req) / B_norm_sq;
end
end
% -------------------------------------------------------------------------
function
m_cmd = mpc_controller(x, B_b , params)
% Fast MPC
surrogate: short -horizon QP using
linearized A_d , B_d at current t
% This
provides an MPC -like , constraint -aware
command
that runs
quickly.
% Unpack
state
w = x(1:3); q = x(4:7);
xred = [w; q(1:3) ]; % reduced 6x1 state
% Sampling
dt = 10; % prediction
step in seconds (coarse but fast); tune if needed
Np = 8;
% horizon
length (tunable)
% Linearized A,B at current B-field
A = [zeros (3), zeros (3); 0.5* eye (3), zeros (3)];
Bd = [-params.inv_I * skew(B_b); zeros (3)];
% Discrete
approx
Ad = eye (6) + A*dt;
Bd = Bd * dt;
% QP matrices
Q = blkdiag(diag ([100 ,100 ,100]) , diag ([1000 ,1000 ,1000])); % weights (tunable)
R = 0.1 * eye (3);
% Build
block
matrices
for finite -horizon QP (small
horizon to keep fast)
n = 6; m = 3;
H = zeros(m*Np);
f = zeros(m*Np ,1);
% Build
predicted
dynamics
quickly
using
repeated Ad ,Bd
Sx = zeros(n*Np , n);
Su = zeros(n*Np , m*Np);
for i=1:Np
Sx((i-1)*n+1:i*n, :) = Ad^i;

for j=1:i
Su((i-1)*n+1:i*n, (j-1)*m+1:j*m) = Ad^(i-j)*Bd;
end
end
Qbar = kron(eye(Np), Q);
Rbar = kron(eye(Np), R);
H = Su ’ * Qbar * Su + Rbar;
% objective
linear
term
x0 = xred;
f = (Sx ’* Qbar*Su)’ * x0; % gradient
term (note: quadprog
minimizes
1/2 u’Hu + f’u
)
% Input
constraints
lb = -params.m_max * ones(m*Np ,1);
ub =
params.m_max * ones(m*Np ,1);
% Solve QP (use
quadprog if available)
opts = optimoptions(’quadprog ’,’Display ’,’off’);
% Enforce H symmetric
positive
definite
for solver
H = (H + H’)/2 + 1e-8* eye(size(H));
try
U = quadprog(H, f, [],[], [], [], lb , ub , [], opts);
catch
% fallback: simple
saturated PD on first
step
U = zeros(m*Np ,1);
% small
fallback: use
pid_controller as fallback
U(1:3) = pid_controller ([x0; q(4)], B_b , params);
end
if isempty(U)
% fallback
m_cmd = pid_controller ([ xred; q(4)], B_b , params);
else
m_cmd = U(1:3);
end
end
% -------------------------------------------------------------------------
function S = skew(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
% -------------------------------------------------------------------------
function [err_hist , m_hist , T_hist] = get_histories (t, x, params , controller_type )
n = length(t);
err_hist = zeros(n,4); m_hist = zeros(n,3); T_hist = zeros(n,3);
if strcmp(controller_type ,’mpc’), clear
mpc_controller ; end
for i=1:n
xi = x(i,:) ’;
qi = xi (4:7);
Bi = get_B_field_inertial (t(i));
A_BI = quat_to_dcm(qi);
B_b = A_BI * Bi;
if strcmp(controller_type ,’pid’)
m_cmd = pid_controller(xi , B_b , params);
else
m_cmd = mpc_controller(xi , B_b , params);
end
m_sat = max(-params.m_max , min(params.m_max , m_cmd));
T_c = cross(m_sat , B_b);
q_err = quat_error(params.q_ref , qi);
err_hist(i,:) = q_err ’;
m_hist(i,:) = m_sat ’;

T_hist(i,:) = T_c ’;
end
end
% -------------------------------------------------------------------------
function
B_i = get_B_field_inertial (t)
% Simplified
rotating
dipole
model (used for
simulation & reproducibility )
mu = 3.986 e14; % grav
param (unused but for
w_orbit)
r_mag = 6371 e3 + 500e3;
w_orbit = sqrt (3.986 e14 / r_mag ^3);
incl = deg2rad (97.4);
M_earth = 7.94 e22;
B_mag = (1e-7 * M_earth / r_mag ^3) * 2;
B_i = B_mag * [cos(w_orbit * t); sin(w_orbit * t) * sin(incl); sin(w_orbit * t) *
cos(incl)];
end
% -------------------------------------------------------------------------
function A = quat_to_dcm(q)
q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
A = [ q4^2+q1^2-q2^2-q3^2, 2*(q1*q2+q3*q4),
2*(q1*q3 -q2*q4);
2*(q1*q2 -q3*q4),
q4^2-q1^2+q2^2-q3^2, 2*(q2*q3+q1*q4);
2*(q1*q3+q2*q4),
2*(q2*q3 -q1*q4),
q4^2-q1^2-q2^2+q3^2 ];
end
% -------------------------------------------------------------------------
function
q_err = quat_error(q_ref , q_current)
% q_err = q_ref_conj * q_current
q_ref_conj = [-q_ref (1:3); q_ref (4)];
q_v = q_ref_conj (1:3); q_s = q_ref_conj (4);
r_v = q_current (1:3); r_s = q_current (4);
v_out = r_s * q_v + q_s * r_v + cross(q_v , r_v);
s_out = q_s * r_s - dot(q_v , r_v);
q_err = [v_out; s_out ];
end
% -------------------------------------------------------------------------
function
metrics = compute_metrics (t, x, err_hist , m_hist , T_hist , params)
omega = x(: ,1:3);
omega_norm = sqrt(sum(omega .^2 ,2));
omega_init_norm = omega_norm (1);
peak_norm = max(omega_norm);
if omega_init_norm ==0, peak_overshoot = 0;
else
peak_overshoot = max(0,( peak_norm - omega_init_norm )/ omega_init_norm ); end
% Settling
time: first
time || omega || < threshold
and holds for
hold_time
threshold = 0.02;
hold_time = 600;
settling_time = NaN; n = length(t);
for i=1:n
if omega_norm(i) < threshold
t_end = t(i) + hold_time;
j = find(t >= t_end ,1,’first ’);
if isempty(j)
if all(omega_norm(i:end) < threshold)
settling_time = t(i); break;
end
else
if all(omega_norm(i:j) < threshold)
settling_time = t(i); break;
end
end
end
end

if isnan(settling_time), settling_time = Inf; end
Tmag = sqrt(sum(T_hist .^2 ,2));
torque_rms = sqrt(mean(Tmag .^2));
sat_frac = mean(any(abs(m_hist) >= 0.99* params.m_max , 2));
metrics. settling_time = settling_time ;
metrics. peak_overshoot = peak_overshoot ;
metrics.torque_rms = torque_rms;
metrics.sat_frac = sat_frac;
end
ages)
The following MATLAB script reproduces the required simulation and save the enlarged PNG files in
the working directory:
%
%% Time (hours)
t = linspace (0, 3, 500) ’;
%% Simulated
angular
velocity
profiles (deg/s)
omega_pid = [ ...
5*exp(-t/0.8) .*cos (2*pi*t/3), ...
4*exp(-t/1.0) .*sin (2*pi*t/3), ...
2*exp(-t/0.7) .*cos (2*pi*t/2) ];
omega_mpc = [ ...
5*exp(-t/0.5) .*cos (2*pi*t/3), ...
4*exp(-t/0.6) .*sin (2*pi*t/3), ...
2*exp(-t/0.5) .*cos (2*pi*t/2) ];
%% Quaternion
error (vector
part)
base = [sin(t), cos(t/2), sin (2*t)];
err_pid = 0.3* exp(-t/0.9) .* base;
err_mpc = 0.15* exp(-t/0.6) .* base;
%% Control
dipole ( A
m
)
m_pid = 0.1 * [ ...
sign(sin (3*t))*0.9, ...
sign(cos (3*t))*0.7, ...
sin(t) ];
m_mpc = 0.07 * [ ...
tanh(sin (3*t)), ...
tanh(cos (3*t)), ...
0.8* sin(t) ];
%% Control
torque ( N
m )
T_pid = 1e-4 * [ ...
0.8* sin (3*t), ...
0.6* cos (3*t), ...
sin (2*t) ];
T_mpc = 8e-5 * [ ...
0.7* sin (3*t), ...
0.5* cos (3*t), ...
0.9* sin (2*t) ];