% generate trajectories based on the external way point sent by the sbpl
function [traj_x, traj_y, traj_times] = plan_traj_poses(poses,cur_pos)
	% prepare the time and points
	speed = 0.01;
	num_pt = numel(poses);
	num_pt = min(num_pt,8); % limit the number of points
	traj_times = ones(num_pt-1,1);
	xpoints = ones(2,num_pt)*speed;
	ypoints = ones(2,num_pt)*speed;
	for i=1:num_pt
		xpoints(1,i) = poses{i}.position.x;
		ypoints(1,i) = poses{i}.position.y;
	end

	% generate traj
	traj_x=gen_traj_dp(10,4,xpoints,traj_times,3);
	traj_y=gen_traj_dp(10,4,ypoints,traj_times,3);
end