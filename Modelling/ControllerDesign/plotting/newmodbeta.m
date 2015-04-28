function  betaDot = newmodbeta(u)


omega = u(1:3,:); 
beta = u(4:6,:);

r = beta(1);
p = beta(2);
y = beta(3);


T = [1, 0, -sin(p);
     0, cos(r), cos(p)*sin(r);
     0, -sin(r), cos(p)*cos(r)];

betaDot = T\omega;

