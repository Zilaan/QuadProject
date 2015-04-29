% Run Three_torque_input.m before you run this file

clc

sim('Three_torques_inputs.slx');

nonlin_vel = squeeze(nonlin_vel);
nonlin_ang = squeeze(nonlin_ang);
lin_vel = lin_vel';
lin_ang = lin_ang';
t = t';
ref = ref';

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

r_ref = ref(1, :);
phi_ref = ref(2, :);
theta_ref = ref(3, :);



%%

figure(1)
clf
hold on
title('Simulation of linear and nonlinear roll')
plot(t, phi_l);
plot(t, phi_nl, '--');
plot(t, phi_ref);
legend('Linear', 'Nonlinear', 'Reference');
xlabel('Time [s]');
ylabel('Angle [rad]');

figure(2)
clf
hold on
title('Simulation of linear and nonlinear pitch')
plot(t, theta_l);
plot(t, theta_nl, '--');
plot(t, theta_ref);
legend('Linear', 'Nonlinear', 'Reference');
xlabel('Time [s]');
ylabel('Angle [rad]');


figure(3)
clf
hold on
title('Simulation of linear and nonlinear yaw rate')
plot(t, r_l);
plot(t, r_nl, '--');
plot(t, r_ref);
legend('Linear', 'Nonlinear', 'Reference');
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');

figure(4)
clf
hold on
title('Simulation of linear and nonlinear yaw')
plot(t, psi_l);
plot(t, psi_nl, '--');
plot(t, r_ref);
legend('Linear', 'Nonlinear', 'Reference for yaw rate');
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');