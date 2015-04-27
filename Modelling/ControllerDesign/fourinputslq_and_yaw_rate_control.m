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
Iz= 0.217e-6;
L=  0.046;
k= 2.75e-11;
b=1e-9;
A= eval(As);
B= eval(Bs);
% C= [0 0 1 0 0 0;0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
C= [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
% D= [ 0 0 0  ; 0 0 0 ; 0 0 0 ];
clear As Bs w1 w2 w3 w4   pd rd yd  Ix Iy Iz wx wy wz tx ty tz p r y beta betad Omega ...
    OmegaD W  x y0 wx0 wy0 wz0 f I n p0 r0 k T tz0 tx0 ty0 t 




%%
%make state space
sys = ss(A,B,C,0); 



    %%
    %Check controllability
    co = ctrb(sys);
controllability = rank(co);
 %% 
%%LQR controller roll pitch yaw
%Weights

r= [ 1 1 1 1];
Q  = (C'*C); 
R=diag(r);
Q(4,4)= Q(4,4)*1e12; 
Q(5,5)= Q(5,5)*1e12; 
Q(6,6)= Q(6,6)*1e9; 


 [K,S,E] = lqr(A,B,Q,R) ;
    
 Kr = -inv(C*inv(A-B*K)*B(:,2:4))  ;


    
    
    
    
    
    
    
    
    
    
    
  %% 
% %%LQR controller yaw rate, roll pitch 
% %Ya rate controll 

 Aa=  A(:,1:5); 
 Aa=  Aa(1:5,:); 
Bb= B(1:5,:);
Cc = [0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1 ];



r= 1*[ 1 1 1 1];
R=diag(r);
Q  = (Cc'*Cc); 
R=diag(r);
Q(3,3)= Q(3,3)*1e5; 
Q(4,4)= Q(4,4)*1e12; 
Q(5,5)= Q(5,5)*1e12; 


 Cs = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;  0 0 0 1 0 0;0 0 0 0 1 0];

    [K,Ss,Ee] = lqr(Aa,Bb,Q,R) ;
    
    Kr = -inv(Cc*inv(Aa-Bb*K)*Bb(:,2:4))  ;
    
    
    
    
    
    %%
    %discrete
     ts= 1e-3; 
    sysD= ss(A,B,eye(6),0,ts);
    sys= ss(A,B,eye(6),0);
   
    sysd= ss(Aa,Bb,Cc,0,ts);
   
    
    
  r= 1*[ 1 1 1 1];
Rd=diag(r);
Qd  = (Cc'*Cc); 
Qd(3,3)= Q(3,3)*1e5; 
Qd(4,4)= Q(4,4)*1e12; 
Qd(5,5)= Q(5,5)*1e12; 

[Kd,Ssd,Eed] = dlqr(sysd.a,sysd.b,Qd,Rd) ;
    
    Krd = -inv(Cc*inv(sysd.a-sysd.b*Kd-eye(5))*sysd.b(:,2:4))  ;
    
    
    
    
step(sysD)
figure
step(sys)
    
    