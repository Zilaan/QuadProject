function  Omegad = Omegadynamics_4(u)

omega = u(1:3,:); 

w1 = u(4);
w2 = u(5);
w3 = u(6);
w4 = u(7);


Ix = 0.14e-6; 
Iy = 0.14e-6; 
Iz = 0.217e-6;
L = 0.046;
k = 2.75e-11;
b = 1e-9;

I = diag([Ix, Iy, Iz]);

T_th = L*k*(w2+ w3-w1 -w4);
T_ph = L*k*(w3+ w4-w1 -w2);
T_ps = b*(w1-w2+w3-w4);

t = [T_ph ; T_th; T_ps];

% Omegad = inv(I)*(-cross(omega, I*omega) + t);
Omegad = I\(-cross(omega, I*omega) + t);

end
