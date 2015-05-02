close all;
%% play wiht D* path planning
map = gen_map([3;3],[9;9]);
goal = [15,15];
start=[1,1];
ds = Dstar(map,'quiet');    % create navigation object
tic; ds.plan(goal); toc;       % create plan for specified goal
figure;
p1 = ds.path(start);

map2 = gen_map([3;3],[7;10]);
%ds = Dstar(map2,'quiet');
ds.costmap_set(map2);
tic; ds.plan(goal); toc;
figure;
ds.path(start);