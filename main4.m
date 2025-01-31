% dynamic planing with external ros node+ trajectory generation

%% init
init
gravity = 9.81;
mass = .507;
trims.phi    = 0;
trims.theta  = 0;
trims.psi    = 0; 
trims.thrust = 0;
[Kp_pos,Kd_pos,Kp_att,Kd_att] = getGain();

%% Create the input-output connections to ROS
clear imu_sub cd_pub rpm_pub motor_pub;
% plan
path_sub = ipc_bridge.createSubscriber('geometry_msgs','PoseArray','plan_path');
% quadrotor state estimation
odom_sub = ipc_bridge.createSubscriber('nav_msgs', 'Odometry', 'odom');
imu_sub = ipc_bridge.createSubscriber('sensor_msgs', 'Imu', 'imu');
% quadrotor control
rpm_pub = ipc_bridge.createPublisher('quadrotor_msgs', 'RPMCommand', 'rpm_cmd');
rpm_msg = rpm_pub.empty();
motor_pub = ipc_bridge.createPublisher('std_msgs', 'Bool', 'motors');
motor_msg = motor_pub.empty();
cd_pub = ipc_bridge.createPublisher('quadrotor_msgs', 'CascadedCommand', 'cascaded_cmd');
cd_msg = cd_pub.empty();
for i = 1:10
	odom_sub.read(10,false);
	imu_sub.read(10, false);
    path_sub.read(10,false);
end

%% data recording
samples = 20000;

odom_idx = 1;
pos_odom = zeros(3,samples);
att_odom = zeros(3,samples);
vel_odom = zeros(3,samples);
ang_odom = zeros(3,samples);
odom_time = zeros(1,samples);

imu_idx = 1;
att_imu = zeros(3,samples);
ang_imu = zeros(3,samples);
imu_time = zeros(1,samples);

err_idx=1;
all_pos_err = zeros(3,samples);
all_vel_err = zeros(3,samples);
all_acce_des = zeros(3,samples);
all_err_time = zeros(1,samples);

%% state machine
state = 0;
isFirstSwitch=true;
isFirstHover=true;

%% static desired state
hover_height_des = 0.75;
pos_zero_des = [0; 0; 0];
vel_zero_des = [0; 0; 0];
acce_zero_des = [0; 0; 0];

%% setup & go
msg_wait = 3;
% find robot starting position
odom_msg = odom_sub.read(msg_wait, false);
odom_updated = false;
if ~isempty(odom_msg)
	start_pos = [odom_msg.pose.pose.position.x;
	odom_msg.pose.pose.position.y;
	odom_msg.pose.pose.position.z];
end
% if you aren't connected to vicon, the code will break here 
motor_msg.data = true;
motor_pub.publish(motor_msg);
pause(1);

%% trajectory
traj_scale = 3;

%% main loop
t_end = 500;
tstart = tic;
while toc(tstart)<t_end
    %% Read the generated path
    path_msg = path_sub.read(msg_wait,false);
    path_updated = false;
    if ~isempty(path_msg)
        path_updated = true;
        cur_path = path_msg.poses;
        display('get new path!')
    end   
	%% Read the odom message, return empty if no message after 3 ms
	odom_msg = odom_sub.read(msg_wait, false);
	odom_updated = false;
	if ~isempty(odom_msg)
		odom_time(odom_idx) = toc(tstart);
		pos_odom(:, odom_idx) = ...
            [odom_msg.pose.pose.position.x;
             odom_msg.pose.pose.position.y;
             odom_msg.pose.pose.position.z];
        vel_odom(:, odom_idx) = ...
            [odom_msg.twist.twist.linear.x;
             odom_msg.twist.twist.linear.y;
             odom_msg.twist.twist.linear.z];
        att_odom(:, odom_idx) = ...
            geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));
        ang_vel(:, odom_idx) = ...
            [odom_msg.twist.twist.angular.x; ...
             odom_msg.twist.twist.angular.y; ...
             odom_msg.twist.twist.angular.z];                                                                               
		% for ease of reference, a small struct holding only current values
		myodom.pos = pos_odom(:,odom_idx);            
		myodom.vel = vel_odom(:,odom_idx);            
		myodom.att = att_odom(:,odom_idx);
		myodom.ang = ang_vel(:,odom_idx);            

		odom_idx = odom_idx + 1;
		odom_updated = true;
	end
		ang_des = zeros(3,1);

	%% Read the imu message, return empty if no message after 3 ms
	imu_msg = imu_sub.read(msg_wait, false);
	imu_updated = false;
	if ~isempty(imu_msg)
		imu_time(:,imu_idx) = toc(tstart);
		att_imu(:,imu_idx) = ...
            geometry_utils.RToZYX(geometry_utils.QuatToR(imu_msg.orientation));
        ang_imu(:,imu_idx) = ...
            [imu_msg.angular_velocity.x; ...
             imu_msg.angular_velocity.y; ...
             imu_msg.angular_velocity.z];
		% for ease of reference, a small struct holding only current values        
		myimu.att = att_imu(:,imu_idx);
		myimu.ang = ang_imu(:,imu_idx);

		imu_idx = imu_idx + 1;
		imu_updated = true;            
	end

	%% Controller
	if imu_idx > 1 && imu_updated && odom_idx > 1 && odom_updated
		%% behavior depends on states
        %switch state first
        %then set the desired point or traj
        switch(state)
            %%idle
            case 0
                if isFirstSwitch
                    isFirstSwitch = false;
                    fprintf('-----idle state-----\n');
                    idleTic = tic;

                    %set desire value for idle
                    pos_des = start_pos; 
                    vel_des = vel_zero_des;
                    acce_des = acce_zero_des; 
                end

                if toc(idleTic)>=2
                    state = 1;
                    isFirstSwitch = true;
                end
            %%takeoff    
            case 1
                if isFirstSwitch
                    isFirstSwitch = false;
                    fprintf('-----takeoff state-----\n');

                    %%generate online landing trajectory, may take some time
                    takeoff_z_points = [start_pos(3),start_pos(3)+hover_height_des;0,0];
                    takeoff_scale = ceil(hover_height_des*10);
                    takeoff_traj_z=gen_traj_dp(6,2,takeoff_z_points,1,3);

                    %%prepare for trja
                    takeoffTic = tic;
                    pos_des = start_pos; 
                    vel_des = vel_zero_des;
                    acce_des = acce_zero_des;
                end

                trackTime = toc(takeoffTic);
                %set desire value for track
                [pos_des(3),vel_des(3),acce_des(3)] = ...
                traj_value(takeoff_traj_z,1,trackTime,takeoff_scale);

                if  toc(takeoffTic) >= takeoff_scale
                    state = 2;
                    isFirstSwitch = true;
                end
            %%hover        
            case 2
                if isFirstSwitch
                    isFirstSwitch = false;
                    fprintf('-----hover state-----\n');                    
                    hoverTic = tic;

                    %set desire value for hover
                    pos_des = myodom.pos(:); %maintain current positon
                    vel_des = vel_zero_des;
                    acce_des = acce_zero_des;  
                end

                if toc(hoverTic)>=2
                    if isFirstHover
                        isFirstHover = false;
                        state = 3;
                        isFirstSwitch = true;
                    else
                        state = 4;
                        isFirstSwitch = true;
                    end
                end
            %% !!!!!!!!!!!!!!!!!!!!!!!!!!track!!!!!!!!!!!!!!!!!!!!!!!
            %% ---------------------------------------------------  
            case 3 
                if isFirstSwitch
                    isFirstSwitch = false;
                    isFail = false;
                    fprintf('-----track state-----\n');
                    % prepare the boot strap time and points
                    if ~isempty(cur_path)
                        traj_times = ones(14,1);
                        xpoints = ones(2,15)*0.01;
                        ypoints = ones(2,15)*0.01;
                        xpoints(1,1) = myodom.pos(1);
                        ypoints(1,1) = myodom.pos(2);
                        xpoints(2,1) = 0; ypoints(2,1) = 0;
                        for i=2:15
                            xpoints(1,i) = cur_path{i}.position.x;
                            ypoints(1,i) = cur_path{i}.position.y;
                        end
                        % generate boot strap traj
                        traj_x=gen_traj_dp(10,4,xpoints,traj_times,3);
                        traj_y=gen_traj_dp(10,4,ypoints,traj_times,3);
                        trackTic = tic;
                        trackBoot = tic;
                    else
                        state = 2;
                        isFirstSwitch = true;
                        isFail = true;
                        warning('the trajectory is empty!!!!!');
                    end            
                end
                % update the traj
                if toc(trackBoot)>2 && path_updated && ~isempty(cur_path) && numel(cur_path)>1
                    [traj_x, traj_y, traj_times] = plan_traj_poses(cur_path,myodom.pos);
                    trackTic = tic;
                end
                % set desire value for track
                if ~isFail
                    trackTime = toc(trackTic);
                    [x_des,xd_des,xdd_des] = ...
                        traj_value(traj_x,traj_times,trackTime,traj_scale);
    
                    [y_des,yd_des,ydd_des]= ...
                        traj_value(traj_y,traj_times,trackTime,traj_scale);
    
                    pos_des = [x_des; y_des; start_pos(3)+hover_height_des]; 
                    vel_des = [xd_des; yd_des; 0];
                    acce_des = [xdd_des; ydd_des; 0];
                end

                %% transit to next state    
                if numel(cur_path)<2
                    state = 2;
                    isFirstSwitch = true;
                end
            %%land        
            case 4
                if isFirstSwitch
                    isFirstSwitch = false;
                    fprintf('-----land state-----\n');                    

                    %%generate online landing trajectory, may take some time
                    land_z_points = [myodom.pos(3),start_pos(3);0,0];
                    land_scale = ceil((myodom.pos(3)-start_pos(3))*10);
                    land_traj_z=gen_traj_dp(6,2,land_z_points,1,3);

                    %%prepare for trja
                    landTic = tic;
                    pos_des = myodom.pos;
                    vel_des = vel_zero_des;
                    acce_des = acce_zero_des; 
                end

                trackTime = toc(landTic);
                %set desire value for track
                [pos_des(3),vel_des(3),acce_des(3)] = ...
                traj_value(land_traj_z,1,trackTime,land_scale);

                if  toc(landTic) >= land_scale
                    state = 5;
                    isFirstSwitch = true;
                end
            %%finish    
            case 5
                if isFirstSwitch
                    isFirstSwitch = false;
                    fprintf('-----finish state-----\n');                    
                    finishTic = tic;

                    %set desire value for idle
                    pos_des = myodom.pos;
                    vel_des = vel_zero_des;
                    acce_des = acce_zero_des;  
                end

                if toc(finishTic)>=2
                    fprintf('-----exit now-----\n');
                    break;
                end

            otherwise
                fprintf('invalid state \n'); 
        end 

		elapsed_t = toc(tstart); 
		
		%% position controller -> here   
		e_pos = myodom.pos - pos_des;
		e_vel = myodom.vel - vel_des;
		u_pos = acce_des - Kp_pos*e_pos - Kd_pos*e_vel;
		uT = mass*(u_pos(3) + trims.thrust + gravity);
		th_cmd = uT;

		%% altitude controller -> onboard
		phi = myodom.att(1); theta = myodom.att(2); psi = myodom.att(3);
		psi_des = 0; % no heading control
		ang_des = zeros(3,1);
		u_r = (u_pos(1)*sin(psi) - u_pos(2)*cos(psi))/gravity;
		u_p = (u_pos(1)*cos(psi) + u_pos(2)*sin(psi))/gravity;
		u_y = psi_des;

        %% record the error
        all_err_time(1,err_idx) = elapsed_t;
        all_pos_err(:,err_idx) = e_pos;
        all_vel_err(:,err_idx) = e_vel;
        all_acce_des(:,err_idx) = acce_des;
        err_idx = err_idx + 1;

		%% send message 
		cd_msg.header.stamp = elapsed_t;
		cd_msg.current_heading = psi;
		cd_msg.thrust = th_cmd;
		e_q = rpy2quat(u_r, u_p, u_y);            
		q_out.x = e_q(2);
		q_out.y = e_q(3);
		q_out.z = e_q(4);
		q_out.w = e_q(1);
		cd_msg.orientation = q_out;
		av_out.x = ang_des(1);
		av_out.y = ang_des(2);
		av_out.z = ang_des(3);
		cd_msg.angular_velocity = av_out;            
		kr = Kp_att;
		Kr.x = kr(1); 
		Kr.y = kr(5); 
		Kr.z = kr(9);  
		cd_msg.kR = Kr;            
		ko = Kd_att;
		kOm.x = ko(1); 
		kOm.y = ko(5); 
		kOm.z = ko(9);          
		cd_msg.kOm = kOm;            
		cd_pub.publish(cd_msg);            


	end
end
cd_msg.thrust = 0;
cd_msg.current_heading = psi;
cd_pub.publish(cd_msg);
motor_msg.data = false;
motor_pub.publish(motor_msg);
%%
clear odom_sub imu_sub cd_pub motor_pub
