function  betad = Betadynamics(u)

omega = u(1:3,:); 
beta = u(4:6,:);

phi = beta(1);
theta = beta(2);
psi = beta(2);


T = [1, 0, -sin(theta);
     0, cos(phi), cos(theta)*sin(phi);
     0, -sin(phi), cos(theta)*cos(phi)];

betad = T\omega;

end