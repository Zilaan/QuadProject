%% LQ control with the three control torque generated using state feedback

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

% Linearization

syms Ix Iy Iz tx ty tz p q r w1 w2 w3 w4 t1 L k b T_ph T_th T_ps
syms phi theta psi

I = diag([Ix, Iy, Iz]);

t = [T_ph; T_th; T_ps];

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
    for n = 1:length(t)
        
        Bs(k, n) = diff(f(k), t(n));
        
    end
end

%% Numeric evaluation using the known data

p = 0; q = 0; r = 0;
phi = 0; theta = 0; psi = 0; 

Ix = 0.14e-6; 
Iy = 0.14e-6; 
Iz = 0.217e-6;
L = 0.046;
k = 2.75e-11;
b = 1e-9;
A =  eval(As);
B = eval(Bs);
C = eye(6); 
D = 0; 
clear As Bs w1 w2 w3 w4   pd rd yd  Ix Iy Iz wx wy wz tx ty tz p r y beta betad Omega ...
    OmegaD W  x y0 wx0 wy0 wz0 f I n p0 r0 k T tz0 tx0 ty0 t  T_ph T_ps T_th t1 


%% Make state space

sys = ss(A,B,C,0); 

%% Check controllability

co = ctrb(sys);
controllability = rank(co);

%% LQR controller yaw rate, roll pitch 
% We must first remove any trace of the yaw to be able to control yaw rate
% .... so we get a new system

Aa = A(:,1:5);
Aa = Aa(1:5,:);
Bb = B(1:5,:);
Cc = [0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1 ]; %outputs to be used in the Reference tracking

% Weights
phi = 1e12*[1 1 1];
R = diag(phi);
Q = (Cc'*Cc); 
Q(3,3) = Q(3,3);  %  yaw rate weight
Q(4,4) = Q(4,4); %  roll weight
Q(5,5) = Q(5,5); %  pitch weight

% Selector matrix to feedback only the 5 states without the yaw from the
% complete model
Cs = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;  0 0 0 1 0 0;0 0 0 0 1 0];

[K,S,E] = lqr(Aa,Bb,Q,R) ;
    
Kr = -inv(Cc*inv(Aa-Bb*K)*Bb) ; 



