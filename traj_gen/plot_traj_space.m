function plot_traj_space(traj1, traj2, times)

rate = 100;
numOfV = rate*sum(times);
v1 = zeros(numOfV,1);
v2 = zeros(numOfV,1);

time = 0:numOfV-1;
time = time/rate;

for i = 1:numOfV
    v1(i) = traj_value(traj1,times,time(i));
    v2(i) = traj_value(traj2,times,time(i));
end    

plot(v1,v2);
axis equal;

end