function q = rpy2quat(phi, theta, psi)
% input: roll, pitch, yaw
% output: [q0 = magnitude, q1, q2, q3 = vector], as a 4x1

q0 =  cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
q1 =  cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
q2 =  cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
q3 = -cos(psi/2)*sin(theta/2)*sin(phi/2) + sin(psi/2)*cos(theta/2)*cos(phi/2);

q = zeros(4,1);
q(1) = q0; 
q(2) = q1; 
q(3) = q2; 
q(4) = q3;

end