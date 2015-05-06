%% initialize paths
clc; clear all; close all;
set(0,'DefaultFigureWindowStyle','docked') % auto-dock figure windows
% if you're in ubuntu, sometimes two fingered scroll & matlab don't play
% nice, so run the following line:
!synclient HorizTwoFingerScroll=0 

% add paths
s=mfilename('fullpath');
pth1 = s(1:end-61);
% pth=s(1:findstr(s,'cmu_quad_matlab')-1); 
addpath([fileparts(s),'/helper_fcns'])
addpath([pth1,'cmu_quad_matlab/dry/src/geometry_utils'])
addpath([pth1,'cmu_quad_matlab/dry/src/quadrotor_model'])
addpath([pth1,'cmu_quad_matlab/dry/install_isolated/share/ipc_bridge/matlab'])
addpath([pth1,'cmu_quad_matlab/wet/build/lib']);

%% model & gains

gravity = 9.81;
mass = .507;

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

trims.phi    = 0;
trims.theta  = 0;
trims.psi    = 0; 
trims.thrust = 0;

Kp_pos = Kp(1:3,1:3);
Kd_pos = Kd(1:3,1:3);
Kp_att = Kp(4:6,4:6);
Kd_att = Kd(4:6,4:6);

%% Create the input-output connections to ROS
clear imu_sub cd_pub rpm_pub motor_pub;

odom_sub = ipc_bridge.createSubscriber('nav_msgs', 'Odometry', 'odom');
imu_sub = ipc_bridge.createSubscriber('sensor_msgs', 'Imu', 'imu');
rpm_pub = ipc_bridge.createPublisher('quadrotor_msgs', 'RPMCommand', 'rpm_cmd');
rpm_msg = rpm_pub.empty();
motor_pub = ipc_bridge.createPublisher('std_msgs', 'Bool', 'motors');
motor_msg = motor_pub.empty();
cd_pub = ipc_bridge.createPublisher('quadrotor_msgs', 'CascadedCommand', 'cascaded_cmd');
cd_msg = cd_pub.empty();

% Clear pending messages
for i = 1:10
	odom_sub.read(10,false);
	imu_sub.read(10, false);
end

%% setup & go
msg_wait = 3;

imu_idx = 1;
odom_idx = 1;

hover_height = .75;    

% find robot starting position
odom_msg = odom_sub.read(msg_wait, false);
odom_updated = false;
if ~isempty(odom_msg)
	start_pos = [odom_msg.pose.pose.position.x;
	odom_msg.pose.pose.position.y;
	odom_msg.pose.pose.position.z];
end

% if you aren't connected to vicon, the code will break here 
% set up a timeline & waypoints
timeline  = [0 3 10 13];
waypoints = [ start_pos, ...
			start_pos+[0;0;hover_height], ...
			start_pos+[0;0;hover_height], ...
			start_pos ];

motor_msg.data = true;
motor_pub.publish(motor_msg);
pause(1);

t_end = timeline(end);
tstart = tic;

psi_des = [];
imu_yaw_deviation = 0;

while toc(tstart)<t_end   
	%% Read the odom message, return empty if no message after 3 ms
	odom_msg = odom_sub.read(msg_wait, false);
	odom_updated = false;
	if ~isempty(odom_msg)

		odom_time(odom_idx) = toc(tstart);

		pos_odom(:,odom_idx) = [odom_msg.pose.pose.position.x;
		odom_msg.pose.pose.position.y;
		odom_msg.pose.pose.position.z];
		vel_odom(:,odom_idx) = [odom_msg.twist.twist.linear.x;
		odom_msg.twist.twist.linear.y;
		odom_msg.twist.twist.linear.z];

		att_odom(:,odom_idx) = ...
		geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));

		ang_vel(:,odom_idx) = [odom_msg.twist.twist.angular.x; ...
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
	%% Read the imu message, return empty if no message after 3 ms
	imu_msg = imu_sub.read(msg_wait, false);
	imu_updated = false;
	if ~isempty(imu_msg)

		imu_time(:,imu_idx) = toc(tstart);

		att_imu(:,imu_idx) = ...
		geometry_utils.RToZYX(geometry_utils.QuatToR(imu_msg.orientation));

		ang_imu(:,imu_idx) = [imu_msg.angular_velocity.x; ...
		imu_msg.angular_velocity.y; ...
		imu_msg.angular_velocity.z];

		% for ease of reference, a small struct holding only current values        
		myimu.att = att_imu(:,imu_idx);
		myimu.ang = ang_imu(:,imu_idx);

		imu_idx = imu_idx + 1;
		imu_updated = true;            
	end
	%% if imu updated...
	if imu_idx > 1 && imu_updated && odom_idx > 1
		elapsed_t = toc(tstart); 
		phi = myodom.att(1); theta = myodom.att(2); psi = myodom.att(3);

		%% your position controller here
		if elapsed_t <= timeline(2)
			next = 2;
		elseif elapsed_t <= timeline(3)
			next = 3;
		else
			next = 4;
		end 
		last = next-1;            
		timescale = timeline(next) - timeline(last);
		dt = elapsed_t-timeline(last);
		last_waypoint = waypoints(:,last);
		next_waypoint = waypoints(:,next);
		[pos_des,vel_des] = poly5_traj(dt,timescale,last_waypoint,next_waypoint);

		%% ye olde linearized controller    
		if isempty(psi_des)
			psi_des = psi;
		end

		ang_des = zeros(3,1);
		e_pos = myodom.pos - pos_des;
		e_vel = myodom.vel - vel_des;

		u_pos = -Kp_pos*e_pos - Kd_pos*e_vel; 
		u_r = (u_pos(1)*sin(psi) - u_pos(2)*cos(psi))/gravity;
		u_p = (u_pos(1)*cos(psi) + u_pos(2)*sin(psi))/gravity;
		u_y = psi_des;

		uT = mass*(u_pos(3) + trims.thrust + gravity);        
		th_cmd = uT;

		% send message 
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
