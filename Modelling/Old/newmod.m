


function  X= newmod(u)


omega= u(1:3,:); 
% beta= u(4:6,:); 
w1= u(4);
w2= u(5);
w3= u(6);
w4= u(7);

% omega,beta, w1,w2,w3,w4

Ix = 0.14e-6; 
Iy= 0.14e-6; 
Iz= 0.217e-6;
L=  0.046;
k= 2.75e-11;
b=1e-9;
% m=26e-3; 
% ir= 1;
% g=9.81;


I = diag([Ix, Iy, Iz]);





% 
% ph= q(1); 
% th=q(2);
% ps=q(3);
% 
% 
% 
% ph_dot= qdot(1);
% th_dot= qdot(2);
% ps_dot= qdot(3);
% 
% 
% w1= f(1); 
% w2= u(2); 
% w3= u(3); 
% w4= u(4); 





% r=beta(1);
% p=beta(2);
% y=beta(2);







% WT=  w1-w2+w3-w4;
T_th= L*k*(w2^2+ w3^2-w1^2 -w4^2);
T_ph=L*k*(w3^2+ w4^2-w1^2 -w2^2);
T_ps= b*(w1^2-w2^2+w3^2-w4^2);
% T= k*(w1^2+w2^2+w3^2+w4^2); 


t = [T_th; T_ph; T_ps];

Omegad = inv(I)*(-cross(omega, I*omega) + t);

% T = [1, 0, -sin(p);
%      0, cos(r), cos(p)*sin(r);
%      0, -sin(r), cos(p)*cos(r)];
% 
% betad = inv(T)*omega;

 X= Omegad;

% R_TB= (T/m)*[  cos(ps)*sin(th)*cos(ph)+sin(ps)*sin(ph); ...
%               sin(ps)*sin(th)*cos(ph)- cos(ps)*sin(ph); ...
%                  cos(th)*cos(ph)] ;
% % x_ddot= R_TB(1);
% % y_ddot= R_TB(2);
% z_ddot= -g+ R_TB(3);
% % state_tr= [ x_ddot; y_ddot; z_ddot];



 
%    wn_inv= [ 1 sin(ph)*tan(th)  cos(ph)*tan(th); ...
%             0   cos(ph)   -sin(ph); ...
%             0   sin(ph)/cos(th)   cos(ph)/cos(th)];
%         eta_dot= [ ph_dot; th_dot; ps_dot];
% 
% 
%   v=  [1, 0  ,-sin(th); 0, cos(ph) , cos(th)*sin(ph) ; 0 ,-sin(ph) ,cos(th)*cos(ph)] * eta_dot;
%   
%   
%   
%   p= v(1);
%   q=v(2);
%   r=v(3);
% 
% 
% 
% 
% % v_dot= [p_dot; q_dot; r_dot];
% v_dot= [ (iyy-izz)*q*r/ixx; (izz-ixx)*p*r/iyy; (ixx-iyy)*p*q/izz]- ...
%        ir*[q/ixx; -p/iyy; 0]*WT + [T_ph/ixx; T_th/iyy; T_ps/izz];
%    
%         
%         
%       
%         
% %          v= [p; q; r];
%    
%    
%     eta_ddot= [0 , ph_dot*cos(ph)*tan(th)+ th_dot*sin(ph)/(cos(th))^2 , - ph_dot*...
%      sin(ph)*cos(th)+th_dot*cos(ph)/cos(th)^2; 0, -ph_dot*sin(ph),  -ph_dot*cos(ph);...
%      0,  ph_dot*cos(ph)/cos(th)+ ph_dot*sin(ph)*tan(th)/cos(th),...
%       -ph_dot*sin(ph)/cos(th) + th_dot*cos(ph)*tan(th)/cos(th)]* v+  wn_inv*v_dot; 
  
  
  
  
 
  
  
  
  

   
   
   





 

