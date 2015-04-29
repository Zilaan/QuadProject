% LQ control with the three control torque generated using state feedback


% Linearization

clc, clear


syms Ix Iy Iz wx wy wz tx ty tz p r y w1 w2 w3 w4 t1 L k b T_ph T_th T_ps
syms pd rd yd 

% r = phi
% p = theta
% y = psi

beta = [r; p; y];
betad = [rd; pd; yd];
I = diag([Ix, Iy, Iz]);



t = [T_ph; T_th; T_ps];


W = [wx; wy; wz];

OmegaD = inv(I)*(-cross(W, I*W) + t);

T = [1, 0, -sin(p);
     0, cos(r), cos(p)*sin(r);
     0, -sin(r), cos(p)*cos(r)];

betad = inv(T)*W;

%%




f = [OmegaD; betad];
x = [wx, wy, wz, r, p, y].';

for k = 1:length(f) % A matrix
    for n = 1:length(x)
        
        As(k, n) = diff(f(k), x(n));
        
    end
end

for k = 1:length(f) % A matrix
    for n = 1:length(t)
        
        Bs(k, n) = diff(f(k), t(n));
        
    end
end

%% 
%Numerique evaluation using the known data
wx =0; 
wy=0;  wz=0;  r=0;  p=0;  y=0; 
Ix = 0.14e-6; 
Iy= 0.14e-6; 
Iz= 0.217e-6;
L=  0.046;
k= 2.75e-11;
b=1e-9;
A= eval(As);
B= eval(Bs);
C= eye(6); 
D=0; 
clear As Bs w1 w2 w3 w4   pd rd yd  Ix Iy Iz wx wy wz tx ty tz p r y beta betad Omega ...
    OmegaD W  x y0 wx0 wy0 wz0 f I n p0 r0 k T tz0 tx0 ty0 t 




%%
%make state space
sys = ss(A,B,C,0); 



 %%
  %Check controllability
    co = ctrb(sys);
controllability = rank(co)



%%
% LQR controller yaw rate, roll pitch 
% Yaw rate controll 
% We must first remove any trace of the yaw to be able to control yaw rate
% .... so we get a new system
Aa=  A(:,1:5);
Aa=  Aa(1:5,:);
Bb= B(1:5,:);
Cc = [0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1 ]; %outputs to be used in the Reference tracking


%%

% Weights
r= 1e12*[ 1 1 1];
R=diag(r);
Q = (Cc'*Cc); 
Q(3,3)= Q(3,3);  %  yaw rate weight
Q(4,4)= Q(4,4); %  roll weight
Q(5,5)= Q(5,5); %  pitch weight

% Selector matrix to feedback only the 5 states without the yaw from the
% complete model
Cs = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;  0 0 0 1 0 0;0 0 0 0 1 0];

[K,S,E] = lqr(Aa,Bb,Q,R) ;
    
Kr = -inv(Cc*inv(Aa-Bb*K)*Bb) ; 




