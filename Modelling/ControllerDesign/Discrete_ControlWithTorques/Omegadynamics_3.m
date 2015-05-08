function  Omegad = Omegadynamics_3(u)


omega = u(1:3,:); 

T_th = u(5);
T_ph = u(4);
T_ps = u(6);


Ix = 0.14e-6; 
Iy = 0.14e-6; 
Iz = 0.217e-6;
L = 0.046;
k = 2.75e-11;
b = 1e-9;

I = diag([Ix, Iy, Iz]);

t = [T_ph ; T_th; T_ps];

Omegad = I\(-cross(omega, I*omega) + t);

end