clc, clear

syms Ix Iy Iz wx wy wz tx ty tz p r y
syms pd rd yd 

% r = phi
% p = theta
% y = psi

beta = [r; p; y];
betad = [rd; pd; yd];
I = diag([Ix, Iy, Iz]);

t = [tx; ty; tz];

W = [wx; wy; wz];

OmegaD = -inv(I)*(cross(W, I*W) + t);

T = [1, 0, -sin(p);
     0, cos(r), cos(p)*sin(r);
     0, -sin(r), cos(p)*cos(r)];

betad = inv(T)*W

%%
clc

r0 = 0;
p0 = 0;
y0 = 0;
wx0 = 0;
wy0 = 0;
wz0 = 0;

tx0 = 0;
ty0 = 0;
tz0 = 0;

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

wx =0; 
wy=0;  wz=0;  r=0;  p=0;  y=0; 
Ix = 0.14e-6; 
Iy= 0.14e-6; 
Iz= 0.217e-6;
A= eval(As)
B= eval(Bs)
C= [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]
D= [ 0 0 0 ; 0 0 0 ; 0 0 0 ]
clear  Bs  pd rd yd  Ix Iy Iz wx wy wz tx ty tz p r y beta betad Omega ...
    OmegaD W  x y0 wx0 wy0 wz0 f I n p0 r0 k T tz0 tx0 ty0 t 









