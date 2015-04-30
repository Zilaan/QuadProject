% Run Three_torque_input.m before you run this file

clc

sim('Three_torques_inputs.slx');

nonlin_vel = squeeze(nonlin_vel);
nonlin_ang = squeeze(nonlin_ang);
lin_vel = lin_vel';
lin_ang = lin_ang';
t = t';
input = input';

p_nl = nonlin_vel(1, :);
q_nl = nonlin_vel(2, :);
r_nl = nonlin_vel(3, :);

p_l = lin_vel(1, :);
q_l = lin_vel(2, :);
r_l = lin_vel(3, :);

phi_nl = nonlin_ang(1, :);
theta_nl = nonlin_ang(2, :);
psi_nl = nonlin_ang(3, :);

phi_l = lin_ang(1, :);
theta_l = lin_ang(2, :);
psi_l = lin_ang(3, :);


p_in = input(1, :);
q_in = input(2, :);
r_in = input(3, :);




%%


figure(1)
clf
hold on

subplot(3, 2, 1);
plot(t, phi_l);
title('$\phi$', 'Interpreter', 'Latex')
xlabel('Time [s]');
ylabel('Angle [rad]');
subplot(3, 2, 3);
plot(t, theta_l);
title('$\theta$', 'Interpreter', 'Latex')
xlabel('Time [s]');
ylabel('Angle [rad]');
subplot(3, 2, 5);
plot(t, psi_l);
title('$\psi$', 'Interpreter', 'Latex')
xlabel('Time [s]');
ylabel('Angle [rad]');

% figure(2)
% clf
% hold on
subplot(3, 2, 2);
plot(t, phi_nl);
xlabel('Time [s]');
ylabel('Angle [rad]');
title('$\phi$', 'Interpreter', 'Latex')
subplot(3, 2, 4);
plot(t, theta_nl);
xlabel('Time [s]');
ylabel('Angle [rad]');
title('$\theta$', 'Interpreter', 'Latex')
subplot(3, 2, 6);
plot(t, psi_nl);
xlabel('Time [s]');
ylabel('Angle [rad]');
title('$\psi$', 'Interpreter', 'Latex')

figure(3)
clf
hold on
subplot(3, 1, 1);
plot(t, p_in);
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('$\tau_p$', 'Interpreter', 'Latex')
subplot(3, 1, 2);
plot(t, q_in);
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('$\tau_q$', 'Interpreter', 'Latex')
subplot(3, 1, 3);
plot(t, r_in);
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('$\tau_r$', 'Interpreter', 'Latex')