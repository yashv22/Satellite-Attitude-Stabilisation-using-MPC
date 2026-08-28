% Transcribed from Appendix 2 of docs/MPC.pdf

%% --- Plot 1: Angular
Velocity
---
figure(’Visible ’,’off’,’Position ’ ,[100 100 900 600]);
subplot (2,1,1);
plot(t, omega_pid , ’LineWidth ’, 1.2);
title(’PID: Angular
Velocity ’);
xlabel(’Time (hours)’);
ylabel(’Angular
Velocity (deg/s)’);
legend(’\omega_x ’, ’\omega_y ’, ’\omega_z ’);
grid on;
subplot (2,1,2);
plot(t, omega_mpc , ’LineWidth ’, 1.2);
title(’MPC: Angular
Velocity ’);
xlabel(’Time (hours)’);
ylabel(’Angular
Velocity (deg/s)’);
legend(’\omega_x ’, ’\omega_y ’, ’\omega_z ’);
grid on;
saveas(gcf , ’angular_velocity_plot .png’);
close(gcf);
%% --- Plot 2: Attitude
Error
---
figure(’Visible ’,’off’,’Position ’ ,[100 100 900 600]);
subplot (2,1,1);
plot(t, err_pid , ’LineWidth ’, 1.2);
title(’PID: Attitude
Error (Quaternion
Vector
Part)’);
xlabel(’Time (hours)’);
ylabel(’Error (q_{e,v})’);
legend(’q_{ex}’, ’q_{ey}’, ’q_{ez}’);
grid on;
subplot (2,1,2);
plot(t, err_mpc , ’LineWidth ’, 1.2);
title(’MPC: Attitude
Error (Quaternion
Vector
Part)’);
xlabel(’Time (hours)’);
ylabel(’Error (q_{e,v})’);
legend(’q_{ex}’, ’q_{ey}’, ’q_{ez}’);
grid on;
saveas(gcf , ’attitude_error_plot .png’);
close(gcf);
%% --- Plot 3: Control
Input
---
figure(’Visible ’,’off’,’Position ’ ,[100 100 900 600]);
subplot (2,1,1);
plot(t, m_pid , ’LineWidth ’, 1.2);
title(’PID: Control
Input (Dipole
Moment)’);
xlabel(’Time (hours)’);
ylabel(’Dipole ( A
m ^2)’);
legend(’m_x’, ’m_y’, ’m_z’);
grid on;
subplot (2,1,2);
plot(t, m_mpc , ’LineWidth ’, 1.2);
title(’MPC: Control
Input (Dipole
Moment)’);
xlabel(’Time (hours)’);
ylabel(’Dipole ( A
m ^2)’);
legend(’m_x’, ’m_y’, ’m_z’);
grid on;
saveas(gcf , ’control_input_plot .png’);
close(gcf);

%% --- Plot 4: Control
Torque
---
figure(’Visible ’,’off’,’Position ’ ,[100 100 900 600]);
subplot (2,1,1);
plot(t, T_pid , ’LineWidth ’, 1.2);
title(’PID: Applied
Control
Torque ’);
xlabel(’Time (hours)’);
ylabel(’Torque ( N
m )’);
legend(’T_x’,’T_y’,’T_z’);
grid on;
subplot (2,1,2);
plot(t, T_mpc , ’LineWidth ’, 1.2);
title(’MPC: Applied
Control
Torque ’);
xlabel(’Time (hours)’);
ylabel(’Torque ( N
m )’);
legend(’T_x’,’T_y’,’T_z’);
grid on;
saveas(gcf , ’control_torque_plot .png’);
close(gcf);
%% Display
confirmation
disp (" Saved: angular_velocity_plot .png , attitude_error_plot .png , control_input_plot .
png , control_torque_plot .png");
References
[1] J. R. Wertz, Spacecraft Attitude Determination and Control, D. Reidel Publishing Company, 1978.
[2] M. L. Psiaki, “Magnetic torquer attitude control via magnetic field model and optimal control,”
Journal of Guidance, Control, and Dynamics, vol. 24, no. 2, 2001, pp. 386–394.
[3] E. Silani and M. Lovera, “Magnetic spacecraft attitude control: a survey and some new results,”
Control Engineering Practice, vol. 13, no. 3, 2005, pp. 357–371.
[4] V. Kapila, M. L. Psiaki, R. J. Crassidis, “Spacecraft Attitude Control Using Magnetic Actuators,”
Advances in the Astronautical Sciences, vol. 105, 2000.
[5] H. Bang, H. Park, H. Bang, “Model Predictive Control for Nanosatellite Attitude Regulation Using
Magnetorquers,” Acta Astronautica, vol. 117, pp. 278–287, 2015.
[6] A. Bemporad and M. Morari, “Control of systems integrating logic, dynamics, and constraints,”
Automatica, 1999.
[7] D. Q. Mayne, J. B. Rawlings, C. V. Rao, P. O. M. Scokaert, “Constrained model predictive control:
Stability and optimality,” Automatica, 2000.
[8] M. Esit, H. E. Soken, C. Hajiyev, “A Model Predictive Control-Based Magnetorquer-Only Attitude
Control Approach for a Small Satellite,” Journal of Guidance,Control, and Dynamics.