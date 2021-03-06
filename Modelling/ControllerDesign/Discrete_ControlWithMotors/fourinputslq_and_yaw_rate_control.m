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
b = 1.1999e-9;
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


phi = 1e-2*[1 1 1 1 ];

Q = (Cc'*Cc); 
R = diag(phi);

Q(1,1) = 1.15*1e5;
Q(2,2) = 1.15*1e5;
Q(3,3) = Q(3,3)*1e7;  % Yaw rate
Q(4,4) = 9*Q(4,4)*1e6;  % Roll
Q(5,5) = 9*Q(5,5)*1e6;  % Pitch






% Selector matrix, to exclude the yaw from the closed loop system
Cs = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;  0 0 0 1 0 0;0 0 0 0 1 0];
Cc1 = [1 1 0 0 0;0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1 ];
[Kd,Sd,Ed] = dlqr(Aa,Bb,Q,R);


Kd
%%
%KR calculation multiple solution so we need to use pinv 
%reffered to:   http://www.academia.edu/6945404/Undergraduate_Lecture_Notes_on_LQG_LQR_controller_design
%note the differnce we are in discret so I modifed the equation from the
%PDF to match a discrete model :)
FF = pinv ([ (Aa-eye(5)) Bb;Cc zeros(3,4)]);
m=4; l=3; n=5;
F= FF(1:n, (size(FF,2)-l+1):size(FF,2));
N= FF((size(FF,1)-m+1):size(FF,1), (size(FF,2)-l+1):size(FF,2) );

Kr= (Kd*F +N)
 

%% Compile and flash the Crazyflie code
clc
user = 'Raman';
if strcmp(user, 'Daniel') || strcmp(user, 'Raman')
    writeC(-Kd, -Kr, user);
end

if strcmp(user, 'Daniel')
    % Daniel
    cd ~/CrazyFlieStuff/crazyflie-firmware/ 
    system('./run.sh');
    cd ~/CHALMERS/EmbeddedSystems/QuadProject/Modelling/ControllerDesign/Discrete_ControlWithMotors
elseif strcmp(user, 'Raman')
    % Raman
    cd ~/Documents/Programmering/Chalmers/Embedded/Project/crazyflie-firmware/ % Raman
    system('./run.sh');
    cd ~/Documents/Programmering/Chalmers/Embedded/QuadProject/Modelling/ControllerDesign/Discrete_ControlWithMotors/
else
    disp('============================================')
    disp('No upload')
    disp('============================================')
end