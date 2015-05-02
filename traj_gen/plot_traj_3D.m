function plot_traj_3D(traj1,traj2,traj3,times)

rate = 100;
numOfV = rate*sum(times);
v1 = zeros(numOfV,1);
v2 = zeros(numOfV,1);
v3 = zeros(numOfV,1);

time = 0:numOfV-1;
time = time/rate;

for i = 1:numOfV
    v1(i) = traj_value(traj1,times,time(i));
    v2(i) = traj_value(traj2,times,time(i));
    v3(i) = traj_value(traj3,times,time(i));
end    

plot3(v1,v2,v3);
axis equal;

end