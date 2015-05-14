%% LQ control with the four PWM generated using state feedback
clc, clear


% roll = phi    (x-axis, facing forward)
% pitch = theta (y-axis, facing left)
% yaw = psi     (z-axis, facing up)

% p = angular velocity around bodyframe x-axis
% q = angular velocity around bodyframe y-axis
% r = angular velocity around bodyframe z-axis

% beta = [phi; theta; psi]
% betad = [phiDot; thetaDot; psiDot]
% omega = [p; q; r]
% omegad = [pDot; qDot; rDot]

% Linerization

syms Ix Iy Iz phi theta psi tx ty tz p q r w1 w2 w3 w4 t1 L k b

I = diag([Ix, Iy, Iz]);

% T_ph = L*k*(w3 + w4 - w1 - w2);   % Old incorrect model
% T_th = L*k*(w2 + w3 - w1 - w4);
% T_ps = b*(w1 - w2 + w3 - w4);

T_ph = L*k*(w3 + w4 - w1 - w2); % New correct model
T_th = L*k*(w2 + w3 - w1 - w4);
T_ps = b*(-w1 + w2 - w3 + w4);

t = [T_ph; T_th; T_ps];
w = [w1; w2; w3; w4]; 

W = [p; q; r];

OmegaD = inv(I)*(-cross(W, I*W) + t);

T = [1, 0, -sin(theta);
     0, cos(phi), cos(theta)*sin(phi);
     0, -sin(phi), cos(theta)*cos(phi)];

betad = inv(T)*W;


f = [OmegaD; betad];
x = [p, q, r, phi, theta, psi].';

for k = 1:length(f) % A matrix
    for n = 1:length(x)
        
        As(k, n) = diff(f(k), x(n));
        
    end
end

for k = 1:length(f) % B matrix
    for n = 1:length(w)
        
        Bs(k, n) = diff(f(k), w(n));
        
    end
end

p = 0; q = 0; r = 0;
phi = 0; theta = 0; psi = 0;

Ix = 0.14e-6;
Iy = 0.14e-6;
Iz = 0.217e-6;
L =  0.046;
k = 2.3499e-9;
b = 1e-9;
A = eval(As);
B = eval(Bs);
% C= [0 0 1 0 0 0;0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
C = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
% D= [ 0 0 0  ; 0 0 0 ; 0 0 0 ];
clear As Bs w1 w2 w3 w4   pd rd yd  Ix Iy Iz wx wy wz tx ty tz p r y beta betad Omega ...
    OmegaD W  x y0 wx0 wy0 wz0 f I n p0 r0 k T tz0 tx0 ty0 t 


%% Make state space
% 
sys = ss(A,B,C,0); 

%% Check controllability

co = ctrb(sys);
controllability = rank(co);


%% Discrete LQR controller yaw rate, roll pitch
% Remove all traces of the yaw to control yaw rate
ts=1/250;
sysd = c2d(sys,ts); 
Aa =  sysd.a(:,1:5);
Aa =  Aa(1:5,:); 
Bb = sysd.b(1:5,:);
Cc = [0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1 ];


phi = 0.01*[ 1 1 1 1];
R = diag(phi);
Q = (Cc'*Cc); 
R = diag(phi);
Q(3,3) = Q(3,3)*1e5;  % Yaw rate
Q(4,4) = Q(4,4)*1e6;  % Roll
Q(5,5) = Q(5,5)*1e6;  % Pitch

% Selector matrix, to exclude the yaw from the closed loop system
Cs = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;  0 0 0 1 0 0;0 0 0 0 1 0];

[Kd,Sd,Ed] = dlqr(Aa,Bb,Q,R) ;
    
Kr_d = -inv(Cc*inv(Aa-Bb*Kd-eye(5))*Bb(:,2:4)); % Reference traking for yaw rate, roll and pitch 
%% Compile and flush the Crazyflie code
clc
writeC(-Kd, Kr_d);

% Daniel
cd ~/CrazyFlieStuff/crazy/ 
system('./run.sh');
cd ~/CHALMERS/EmbeddedSystems/QuadProject/Modelling/ControllerDesign/Discrete_ControlWithMotors


% Raman
% cd ~/Documents/Programmering/Chalmers/Embedded/Project/crazyflie-firmware/ % Raman
% system('./run.sh');
% cd ~/Documents/Programmering/Chalmers/Embedded/QuadProject/Modelling/ControllerDesign/Discrete_ControlWithMotors/
