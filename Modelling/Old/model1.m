function  X = model1(u)
	% The angles
    
    
    
ixx = 0.14e-6; 
iyy= 0.14e-6; 
izz= 0.217e-6;
L=  0.046;
k= 2.75e-11;
b=1e-9;
m=26e-3; 
ir= 1;
g=9.81;
    
    
	ph = u(1);
	th = u(2);
	ps = u(3);

	% The angular velocity
	ph_dot = u(7);
	th_dot = u(8);
	ps_dot = u(9);

	w1 = u(13);
	w2 = u(14);
	w3 = u(15);
	w4 = u(16);

	WT=  w1-w2+w3-w4;
T_th= L*k*(w2^2+ w3^2-w1^2 -w4^2);
T_ph=L*k*(w3^2+ w4^2-w1^2 -w2^2);
T_ps= b*(w1^2-w2^2+w3^2-w4^2);

T= k*(w1^2+w2^2+w3^2+w4^2); 



% R_TB= (T/m)*[  cos(ps)*sin(th)*cos(ph)+sin(ps)*sin(ph); ...
%               sin(ps)*sin(th)*cos(ph)- cos(ps)*sin(ph); ...
%                  cos(th)*cos(ph)] ;


x_ddot= cos(ps)*sin(th)*cos(ph)+sin(ps)*sin(ph)*(T/m);
y_ddot= (sin(ps)*sin(th)*cos(ph)- cos(ps)*sin(ph))*(T/m) ;
z_ddot= ( -g+  cos(th)*cos(ph)) *(T/m);
state_tr= [ x_ddot; y_ddot; z_ddot];



 
   wn_inv= [ 1 sin(ph)*tan(th)  cos(ph)*tan(th); ...
            0   cos(ph)   -sin(ph); ...
            0   sin(ph)/cos(th)   cos(ph)/cos(th)];
        
        eta_dot= [ ph_dot; th_dot; ps_dot];


  v=  [1, 0  ,-sin(th); 0, cos(ph) , cos(th)*sin(ph) ; 0 ,-sin(ph) , ...
      cos(th)*cos(ph)] * eta_dot;
  
  
  
  p= v(1);
  q=v(2);
  r=v(3);




% v_dot= [p_dot; q_dot; r_dot];
v_dot= [ (iyy-izz)*q*r/ixx; (izz-ixx)*p*r/iyy; (ixx-iyy)*p*q/izz]- ...
       ir*[q/ixx; -p/iyy; 0]*WT + [T_ph/ixx; T_th/iyy; T_ps/izz];
   
        
        
      
        

   
   
    eta_ddot= [0 , ph_dot*cos(ph)*tan(th)+ th_dot*sin(ph)/(cos(th))^2 , - ph_dot*...
     sin(ph)*cos(th)+th_dot*cos(ph)/cos(th)^2; 0, -ph_dot*sin(ph),  -ph_dot*cos(ph);...
     0,  ph_dot*cos(ph)/cos(th)+ ph_dot*sin(ph)*tan(th)/cos(th),...
      -ph_dot*sin(ph)/cos(th) + th_dot*cos(ph)*tan(th)/cos(th)]* v+  wn_inv*v_dot; 
  

	% Function output
	X = [ eta_ddot;state_tr];
end















