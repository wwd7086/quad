% test path_plan subscribe

%% init
init

%% Create the input-output connections to ROS
clear imu_sub cd_pub rpm_pub motor_pub;
% planner path
path_sub = ipc_bridge.createSubscriber('geometry_msgs','PoseArray','plan_path');
% fan position
fan1 = ipc_bridge.createSubscriber('nav_msgs','Odometry','fan1/odom');
fan2 = ipc_bridge.createSubscriber('nav_msgs','Odometry','fan2/odom');
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
    fan1.read(10,false);
    fan2.read(10,false);
    path_sub.read(10,false);
end

%%try to read path
while true
	path_msg = path_sub.read(10, false);
	if ~isempty(path_msg)
		path_msg		
	end
end