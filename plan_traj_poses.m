% generate trajectories based on the external way point sent by the sbpl
function [traj_x, traj_y, traj_times] = plan_traj_poses(poses,cur_pos)
	% prepare the time and points
	speed = 0.1;
	num_pt = numel(poses)
	
	[x,Y]=poses2array(poses);
	traj = spline(x,Y);
	
	num_pt = min(num_pt,10); % limit the number of points
	traj_times = ones(num_pt-1,1);
	xpoints = ones(2,num_pt)*speed;
	ypoints = ones(2,num_pt)*speed;
	%xpoints(1,1) = (cur_pos(1)+poses{1}.position.x)/2;
	%ypoints(1,1) = (cur_pos(2)+poses{1}.position.y)/2;
	for i=1:num_pt
		xpoints(1,i) = poses{i}.position.x;
		ypoints(1,i) = poses{i}.position.y;
        
        % find proper velocity direction
        p1 = ppval(traj,poses{i}.position.x+0.1);
        p0 = ppval(traj,poses{i}.position.x-0.1);
        speed_n = [0.2,p1-p0]/sqrt(0.04+(p1-p0)^2);
        
		xpoints(2,i) = speed*speed_n(1);
		ypoints(2,i) = speed*speed_n(2);
	end

	% generate traj
	traj_x=gen_traj_dp(10,4,xpoints,traj_times,3);
	traj_y=gen_traj_dp(10,4,ypoints,traj_times,3);
end

function [x,y] = poses2array(poses)
	x = zeros(1,size(poses,2));
	y = zeros(1,size(poses,2));
	for i=1:size(poses,2)
		x(1,i)=poses{i}.position.x;
		y(1,i)=poses{i}.position.y;
    end
end