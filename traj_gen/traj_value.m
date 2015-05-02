function [value, dvalue, ddvalue] = traj_value(traj, times, t, scale)
%% optional scale parameter
if nargin<4
	scale =1;
end

%% check the range of time
t = t/scale;
if t>sum(times)
    %%warning('time exceed limit');
    %%value = 0;
    %%return;
    t = sum(times);
end

%% decide which range it fits in
i = 1;
while t-times(i)>0
    t = t-times(i);
    i = i+1;
end

%look into the ith traj, with t
order = size(traj,1)/size(times,1);
p = traj(order*(i-1)+1:order*i,1);
p = fliplr(p');

dp = polyder(p/scale);
dpp = polyder(dp/scale);

value = polyval(p,t);
dvalue = polyval(dp,t);
ddvalue = polyval(dpp,t);

end