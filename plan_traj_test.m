% test the plan_traj function
close all;

map_scale = 3;
traj_scale = 0.8;
start_pos = [0,0];
goal_pos = [1.8,2.5];
fan1_pos = [-1,0];
fan2_pos = [1,1];

[traj_x, traj_y, traj_times] = plan_traj(...
    start_pos, goal_pos, fan1_pos, fan2_pos, map_scale);

show_plan_traj(traj_x, traj_y, traj_times, fan1_pos, fan2_pos);