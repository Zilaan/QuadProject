
%%
% %
% LQI control with the four PWV  generated using state feedback





%%

% linerization
clc, clear

syms Ix Iy Iz wx wy wz tx ty tz p r y w1 w2 w3 w4 t1 L k b
syms pd rd yd 

% r = phi
% p = theta
% y = psi

beta = [r; p; y];
betad = [rd; pd; yd];
I = diag([Ix, Iy, Iz]);

T_th= L*k*(w2+ w3-w1 -w4);
T_ph=L*k*(w3+ w4-w1 -w2);
T_ps= b*(w1-w2+w3-w4);


t = [T_ph; T_th; T_ps];
t1= [ w1; w2; w3; w4]; 

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
    for n = 1:length(t1)
        
        Bs(k, n) = diff(f(k), t1(n));
        
    end
end

wx =0; 
wy=0;  wz=0;  r=0;  p=0;  y=0; 



Ix = 0.14e-6; 
Iy= 0.14e-6; 
b=1e-9;
Iz= 0.217e-6;
L=  0.046;
k= 2.75e-11;


% L=  0.046/sqrt(2);
% k= 2.75e-9; 
% 
% 
% 
% Ix = 6.228e-3;
% Iy = 6.228e-3;
% Iz = 1.121e-2;
% b= 1.0942e-7*(pi/30)^2; 
% 


A= eval(As);
B= eval(Bs);
% C= [0 0 1 0 0 0;0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
C= [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
% D= [ 0 0 0  ; 0 0 0 ; 0 0 0 ];
clear As  w1 w2 w3 w4   pd rd yd  Ix Iy Iz wx wy wz tx ty tz p r y beta betad Omega ...
    OmegaD W  x y0 wx0 wy0 wz0 f I n p0 r0 k T tz0 tx0 ty0 t 




%%
%make state space
sys = ss(A,B,C,0);
ts=1/250;

sysd= c2d(sys,ts);

    %%
    %Check controllability
    co = ctrb(sys);
controllability = rank(co);


    
    
  n=2;   
    
    
    
    
    
  %% 
% %%LQI controller yaw rate, roll pitch 
% %Ya rate controll 
% remove all traces of the yaw to control yaw rate


 
 Aa=  sys.a(:,1:5); 
 Aa=  Aa(1:5,:); 
Bb= sys.b(1:5,:);



 Aad=  sysd.a(:,1:5); 
 Aad=  Aad(1:5,:); 
Bbd= sysd.b(1:5,:);


Cc = [0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1 ];

Cs = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;  0 0 0 1 0 0;0 0 0 0 1 0];

%%

r= 1e-9*[ 1 1 1 1];
R=diag(r);

 
 
 
 %%

 
 sys1= ss(Aa, Bb, Cc,0);
Q= eye(8); 

Q(1,1)= 0;  
Q(2,2)= 0;

Q(3,3)= Q(3,3)*0;  %yaw rate
Q(4,4)= Q(4,4)*0;  %roll
Q(5,5)= Q(5,5)*0;  %pitch

Q(6,6)= 1;  
Q(7,7)= 1000000;  
Q(8,8)= 1000000;  
 
Ai=[ Aa zeros(5,3); Cc zeros(3,3)]; 
Bi= [Bb; zeros(3,4)];


Aid=[ Aad zeros(5,3); Cc zeros(3,3)]; 
Bid= [Bbd; zeros(3,4)];


%%
%continuous time
[K,Ss,Ee] = lqr(Ai, Bi,Q,R,0) ;
K


%%
% %Discrte time time

% [Kd,Ss,Ee] = dlqr(,Q,R,0) ;
% Kd



    
    



  


    