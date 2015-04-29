function  omegaDot = newmod(u)

% Simulink script for omega dynamics

omega= u(1:3, :);

tx = u(4);
ty = u(5);
tz = u(6);

Ix = 0.14e-6; 
Iy = 0.14e-6; 
Iz = 0.217e-6;

I = diag([Ix, Iy, Iz]);

tau = [tx; ty; tz];

omegaDot = I\( -cross(omega, I*omega) + tau);
