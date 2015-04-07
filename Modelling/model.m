function  X= model(u)









ph= u(1); 
th=u(2);
ps=u(3);



ph_dot= u(4);
th_dot= u(5);
ps_dot= u(6);


w1= u(7); 
w2= u(8); 
w3= u(9); 
w4= u(10); 












WT=  w1-w2+w3-w4;
T_th= L*k(-w2^2+ w4^2);
T_ph=L*k(-w1^2+ w3^2);
T_ps= b(w1^2+w2^2+w3^2+w4^2);



R_TB= (T/m)*[  cos(ps)*sin(th)*cos(ph)+sin(ps)*sin(ph); ...
              sin(ps)*sin(th)*cos(ph)- cos(ps)*sin(ph); ...
                 cos(th)*cos(ph)] ;
x_ddot= R_TB(1);
y_ddot= R_TB(2);
z_ddot= -g+ R_TB(3);
state_tr= [ x_ddot; y_ddot; z_ddot];




% v_dot= [p_dot; q_dot; r_dot];
v_dot= [ (iyy-izz)*q*r/ixx; (izz-ixx)*p*r/iyy; (ixx-iyy)*p*q/izz]- ...
       ir*[q/ixx; -p/iyy; 0]*WT + [T_ph/ixx; T_th/iyy; T_ps/izz];
   
   wn_inv= [ 1 sin(ph)*tan(th)  cos(ph)*tan(th); ...
            0   cos(ph)   -sin(ph); ...
            0   sin(ph)/cos(th)   cos(ph)/cos(th)]; 
        
        eta_dot= [ ph_dot; th_dot; ps_dot];
        v= inv(wn_ivn)* eta_dot;
        
%          v= [p; q; r];
   
   
    eta_ddot= [0  ph_dot*cos(ph)*tan(th)+ th_dot*sin(ph)/(cos(th))^2  - ph_dot*...
     sin(ph)*cos(th)+th_dot*cos(ph)/cos(th)^2; 0 -ph_dot*sin(ph)  -ph_dot*cos(ph);...
     0  ph_dot*cos(ph)/cos(th)+ ph_dot*sin(ph)*tan(th)/cos(th) ...
      -ph_dot*sin(ph)/cos(th) + th_dot*cos(ph)*tan(th)/cos(th)]* v+  wn_inv*v_dot; 
  
  
  
  
  X= [ state_tr; eta_ddot];
  
  
  
  

   
   
   





 

