function plot_traj_time(traj, times)

rate = 100;
numOfV = rate*sum(times);
v = zeros(numOfV,1);
time = 0:numOfV-1;
time = time/rate;

for i = 1:numOfV
    v(i) = traj_value(traj,times,time(i));
end    

plot(time,v);

end