clear all;
close all;

%vel = 2.2;
%xpoints = [0 2 0 -2 0; vel 0 -vel 0 vel];
%ypoints = [0 1 2 1 0; 0 vel 0 -vel 0];
%zpoints = [0 1 1 1 1; 0 0 0 0 0];
%times = [1;1;1;1];

times = ones(10,1);
xpoints = [0 1 2 3 2 1 2 4 2 3 1; 0 1 4 2 0 1 0 2 0 0 0];
ypoints = [0 2 1 3 2 2 3 4 1 2 3; 0 1 3 2 0 3 4 4 1 0 0];
zpoints = [0 1 2 2 3 3 3 3 3 3 3; 0 0 1 0 1 0 0 0 0 0 0];

polyOrder = 10;
derOrder = 4;
contOrder = 5;

xtraj = gen_traj_dp(polyOrder,derOrder,xpoints, times, contOrder);
ytraj = gen_traj_dp(polyOrder,derOrder,ypoints, times, contOrder);
ztraj = gen_traj_dp(6,2,zpoints, times, 3);

%plot_traj_time(ztraj, times);
plot_traj_3D(xtraj,ytraj,ztraj,times);
%plot_traj_space(xtraj,ytraj,times);
