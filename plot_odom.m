%plot the imu and odometry reading
%pos_odom = zeros(3,samples);
%att_odom = zeros(3,samples);
%vel_odom = zeros(3,samples);
%ang_odom = zeros(3,samples);
%odom_time = zeros(1,samples);

%att_imu = zeros(3,samples);
%ang_imu = zeros(3,samples);
%imu_time = zeros(1,samples);
close all;

figure;
plot(odom_time, pos_odom);
title('position from odometry');

figure;
plot(odom_time, vel_odom);
title('velocity from odometry');

figure;
plot(odom_time, att_odom);
title('attitude from odometry');