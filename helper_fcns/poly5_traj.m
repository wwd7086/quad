function [pos_des,vel_des] = poly5_traj(t,tf,strt_p,end_p)

xf = end_p(1);    x0 = strt_p(1);    dx = x0-xf;   
yf = end_p(2);    y0 = strt_p(2);    dy = y0-yf;  
zf = end_p(3);    z0 = strt_p(3);    dz = z0-zf;

x_co = [ -(6*(dx))/tf^5, (15*(dx))/tf^4,  -(10*(dx))/tf^3,  0,  0, x0 ];
y_co = [ -(6*(dy))/tf^5, (15*(dy))/tf^4,  -(10*(dy))/tf^3,  0,  0, y0 ];
z_co = [ -(6*(dz))/tf^5, (15*(dz))/tf^4,  -(10*(dz))/tf^3,  0,  0, z0 ];


pos_des = vertcat(    x_co(1)*t.^5 +    x_co(2)*t.^4 +   x_co(3)*t.^3 +   x_co(4)*t.^2 + x_co(5)*t + x_co(6),...
                      y_co(1)*t.^5 +    y_co(2)*t.^4 +   y_co(3)*t.^3 +   y_co(4)*t.^2 + y_co(5)*t + y_co(6),...
                      z_co(1)*t.^5 +    z_co(2)*t.^4 +   z_co(3)*t.^3 +   z_co(4)*t.^2 + z_co(5)*t + z_co(6));
         
vel_des = vertcat(  5*x_co(1)*t.^4 +  4*x_co(2)*t.^3 + 3*x_co(3)*t.^2 + 2*x_co(4)*t    + x_co(5),...
                    5*y_co(1)*t.^4 +  4*y_co(2)*t.^3 + 3*y_co(3)*t.^2 + 2*y_co(4)*t    + y_co(5),... 
                    5*z_co(1)*t.^4 +  4*z_co(2)*t.^3 + 3*z_co(3)*t.^2 + 2*z_co(4)*t    + z_co(5));        

end