


function  X= newmodbeta(u)


omega= u(1:3,:); 
beta= u(4:6,:);











r=beta(1);
p=beta(2);
y=beta(2);










T = [1, 0, -sin(p);
     0, cos(r), cos(p)*sin(r);
     0, -sin(r), cos(p)*cos(r)];

betad = inv(T)*omega;

 X= betad;

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
  
  
  
  
 
  
  
  
  

   
   
   





 

