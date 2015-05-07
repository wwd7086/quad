%plan and generate the trajectory
function [traj_x, traj_y, traj_times] = plan_traj(start_pos, goal_pos, fan1, fan2, map_scale)
tic;
%world boundary
% xmin xmax ymin ymax
worldRect = [-1.05, 1.85, -1.44, 2.5];

%convert world => map
start_plan = world2map(start_pos,map_scale, worldRect);
goal_plan = world2map(goal_pos,map_scale, worldRect);
fan1_plan = world2map(fan1,map_scale, worldRect);
fan2_plan = world2map(fan2,map_scale, worldRect);

%generate way points
map_size = [ceil(worldRect(2)-worldRect(1))*map_scale,...
            ceil(worldRect(4)-worldRect(3))*map_scale]; 
fan_size = 1;

map = gen_map(fan1_plan,fan2_plan, map_size, fan_size);
ds = Dstar(map,'quiet');
ds.plan(goal_plan);
path = ds.path(start_plan);
%ds.path(start_plan);

%convert map => world
path = map2world(path, map_scale, worldRect);
num_pts = size(path,1);

%generate trajs based on world
traj_times = ones(num_pts+2,1);
second_pos = map2world(start_plan,map_scale,worldRect);
last_pos = bsxfun(@plus, path(end,:), 0.01);
xpoints = [start_pos(1),second_pos(1),path(:,1)',last_pos(1)];%;ones(1,num_pts)];
ypoints = [start_pos(2),second_pos(2),path(:,2)',last_pos(2)];%;ones(1,num_pts)];
traj_x=gen_traj_dp(10,4,xpoints,traj_times,3);
traj_y=gen_traj_dp(10,4,ypoints,traj_times,3);

toc;
end