function [dx] = underwater_Dynamics_Sim(x,u,p,t,vdat) 

x1 = x(:,1); 
x2 = x(:,2); 
x3 = x(:,3); 
x4 = x(:,4); 
x5 = x(:,5); 
x6 = x(:,6);  
x7 = x(:,7); 
x8 = x(:,8); 
x9 = x(:,9);
x10 = x(:,10); 



c_x = 0.5; r_x = 0.1; u_x = 2; c_z = 0.1; u_z = 0.1; 

E = exp(-((x1 - c_x)./r_x).^2); 
R_x = -u_x.*E.*(x1-c_x).*((x3 - c_z)./c_z).^2; 
R_z = -u_z.*E.*((x3-c_z)/c_z).^2; 

dx(:,1) = x7.*cos(x6).*cos(x5) + R_x; 
dx(:,2) = x7.*sin(x6).*cos(x5); 
dx(:,3) = -x7.*sin(x5) + R_z; 
dx(:,4) = x8 + x9.*sin(x4).*tan(x5) + x10.*cos(x4).*tan(x5); 
dx(:,5) = x9.*cos(x4) - x10.*sin(x4); 
dx(:,6) = (x9.*sin(x4)./cos(x5)) + (x10.*cos(x4)./cos(x5));  
dx(:,7) = u(:,1); 
dx(:,8) = u(:,2); 
dx(:,9) = u(:,3); 
dx(:,10) = u(:,4);
