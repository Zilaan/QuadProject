% Run Three_torque_input.m before you run this file

clc

g = 10; % Gain for input torques

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

fz1 = 14;
fz2 = 18;

subplot(3, 2, 1);
p1 = plot(t, phi_l);
title('$\phi$', 'Interpreter', 'Latex','FontSize', fz2)
xlabel('Time [s]','FontSize',fz1);
ylabel('Angle [rad]','FontSize',fz1);
subplot(3, 2, 3);
plot(t, theta_l);
title('$\theta$', 'Interpreter', 'Latex','FontSize',fz2)
xlabel('Time [s]','FontSize', fz1);
ylabel('Angle [rad]','FontSize', fz1);
subplot(3, 2, 5);
plot(t, psi_l);
title('$\psi$', 'Interpreter', 'Latex','FontSize', fz2)
xlabel('Time [s]','FontSize', fz1);
ylabel('Angle [rad]','FontSize', fz1);



h1 = gcf;
h1.Position(3) = 720;
h1.Position(4) = 390;

subplot(3, 2, 2);
plot(t, phi_nl);
xlabel('Time [s]','FontSize', fz1);
ylabel('Angle [rad]','FontSize', fz1);
title('$\phi$', 'Interpreter', 'Latex','FontSize', fz2)
subplot(3, 2, 4);
plot(t, theta_nl);
xlabel('Time [s]','FontSize', fz1);
ylabel('Angle [rad]','FontSize', fz1);
title('$\theta$', 'Interpreter', 'Latex','FontSize', fz2)
subplot(3, 2, 6);
plot(t, psi_nl);
xlabel('Time [s]','FontSize', fz1);
ylabel('Angle [rad]','FontSize', fz1);
title('$\psi$', 'Interpreter', 'Latex','FontSize', fz2)

figure(3)
clf
hold on
subplot(3, 1, 1);
plot(t, p_in);
xlabel('Time [s]', 'FontSize', fz1);
ylabel('Torque [Nm]', 'FontSize', fz1);
title('$\tau_p$', 'Interpreter', 'Latex', 'FontSize', fz2)
subplot(3, 1, 2);
plot(t, q_in);
xlabel('Time [s]', 'FontSize', fz1);
ylabel('Torque [Nm]', 'FontSize', fz1);
title('$\tau_q$', 'Interpreter', 'Latex', 'FontSize', fz2)

axis([0, 14, 0, 1.5*10^(-7)])

subplot(3, 1, 3);
plot(t, r_in);
xlabel('Time [s]', 'FontSize', fz1);
ylabel('Torque [Nm]', 'FontSize', fz1);
title('$\tau_r$', 'Interpreter', 'Latex', 'FontSize', fz2)

h2 = gcf;
h2.Position(3) = 720;
h2.Position(4) = 390;