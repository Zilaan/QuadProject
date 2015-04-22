function  X= motordyn(u)








L=  0.046;
k= 2.75e-11;
b=1e-9;



w1= u(1); 
w2= u(2); 
w3= u(3); 
w4= u(4); 








T_th= L*k*(w2^2+ w3^2-w1^2 -w4^2);
T_ph=L*k*(w3^2+ w4^2-w1^2 -w2^2);
T_ps= b*(w1^2-w2^2+w3^2-w4^2);




% T_th= L*k*(-w2^2+ w4^2);
% T_ph=L*k*(-w1^2+ w3^2);
% T_ps= b*(w1^2+w2^2+w3^2+w4^2);



  X = [T_th; T_ph; T_ps];
  
 
  
  
  

   
   
   





 

