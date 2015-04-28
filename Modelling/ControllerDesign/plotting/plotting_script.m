clc, clear

syms p q r phi theta psi phiDot thetaDot psiDot
syms Ix Iy Iz L k b tx ty tz

% roll = phi    (x-axis, facing forward) 
% pitch = theta (y-axis, facing left)
% yaw = psi     (z-axis, facing up)

% p = angular velocity around bodyframe x-axis
% q = angular velocity around bodyframe y-axis
% r = angular velocity around bodyframe z-axis

W = [p; q; r];
tau = [tx; ty; tz];

I = diag([Ix, Iy, Iz]);

OmegaD = I\(-cross(W, I*W) + tau);

T = [1, 0, -sin(theta);
     0, cos(phi), cos(theta)*sin(phi);
     0, -sin(phi), cos(theta)*cos(phi)];

betad = T\W;

f = [OmegaD; betad];
x = [p, q, r, phi, theta, psi].';

for k = 1:length(f)  % A matrix
    for n = 1:length(x)  % B matrix
        
        As(k, n) = diff(f(k), x(n));
        
    end
end

for k = 1:length(f)  % A matrix
    for n = 1:length(tau)  % B matrix
        
        Bs(k, n) = diff(f(k), tau(n));
        
    end
end

p = 0;
q = 0;
r = 0;

theta = 0;
phi = 0;
psi = 0;

Ix = 0.14e-6;   %  Inertia around x-axis (roll)
Iy = 0.14e-6;   %  Inertia around y-axis (pitch)
Iz = 0.217e-6;  %  Inertia around z-axis (yaw)
L = 0.046;      %  Distance between rotors and axes
k = 2.75e-11;   %  PWM to thrust
b = 1e-9;       %  PWM to torque around yaw axis

A = eval(As);
B = eval(Bs);

clear vars p q r phi theta psi phiDot thetaDot psiDot
clear vars Ix Iy Iz L k b tx ty tz As Bs f I OmegaD T tau W x betad


C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0];


%%   Check controllability
co = ctrb(A, B);
controllability = rank(co);

    
%% 
%  LQR controller yaw rate, roll pitch 
%  Yaw rate controll 

Aa = A(:, 1:5); 
Aa = Aa(1:5, :); 
Bb = B(1:5, :);


%%
r = 1*[1, 1, 1];
Q = eye(5);
R = diag(r);
Q(3,3) = Q(3,3)*1e5; 
Q(4,4) = Q(4,4)*1e12; 
Q(5,5) = Q(5,5)*1e12; 

[K,Ss,Ee] = lqr(Aa,Bb,Q,R);
clc

Cc = [0, 0, 1, 0, 0;
      0, 0, 0, 1, 0;
      0, 0, 0, 0, 1];
Cs = [1, 0, 0, 0, 0, 0;
      0, 1, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 0;
      0, 0, 0, 1, 0, 0;
      0, 0, 0, 0, 1, 0;
      0, 0, 0, 0, 0, 0;];

Kr = (-inv(Cc*inv(Aa-Bb*K)*Bb))';


%%  Plotting
clc
g = 0.1;

clear vars lin_vel lin_ang nonlin_ang nonlin_vel t ref

sim('plotting_sim.slx');

nonlin_ang = squeeze(nonlin_ang);
nonlin_vel = squeeze(nonlin_vel);

%%


figure(1)
clf
hold on
title('Non linear and linear simulation of pitch');
plot(t, lin_ang(:, 1), '--');
plot(t, nonlin_ang(1, 1, :));
plot(t, ref(:, 1),'--');
legend('Linear', 'Non linear', 'Reference');
    
    