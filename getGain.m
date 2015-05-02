%% get the PID gain for controller
function [Kp_pos,Kd_pos,Kp_att,Kd_att] = getGain()

% gains, sorta kinda maybe
Kp = zeros(6); Kd = zeros(6);

% position & velocity gains
Kp(1,1) = 13;
Kp(2,2) = 13;
Kp(3,3) = 20;  

Kd(1,1) = 6;
Kd(2,2) = 6;
Kd(3,3) = 6; 

Kp(4,4) = 232;
Kp(5,5) = 232;
Kp(6,6) = 56.6;

Kd(4,4) = 28.8;
Kd(5,5) = 28.8;
Kd(6,6) = 13.7;

Kp_pos = Kp(1:3,1:3);
Kd_pos = Kd(1:3,1:3);
Kp_att = Kp(4:6,4:6);
Kd_att = Kd(4:6,4:6);

end