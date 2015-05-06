% plot the error
%all_pos_err = zeros(3,samples);
%all_vel_err = zeros(3,samples);
%all_acce_des = zeros(3,samples);
%all_err_time = zeros(1,samples);
close all;

figure;
plot(all_err_time,all_pos_err);
title('position error');

figure;
plot(all_err_time,all_vel_err);
title('velocity error');

figure;
plot(all_err_time,all_acce_des);
title('desired acceleartion');