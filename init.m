%% initialize all the paths and missclineaous
%% initialize paths
clc; clear all; close all;
set(0,'DefaultFigureWindowStyle','docked') % auto-dock figure windows
% if you're in ubuntu, sometimes two fingered scroll & matlab don't play
% nice, so run the following line:
!synclient HorizTwoFingerScroll=0 

% add paths
s=mfilename('fullpath');
pth1 = s(1:end-54);
addpath([fileparts(s),'/helper_fcns'])
addpath([fileparts(s),'/traj_gen'])
addpath([fileparts(s),'/path_plan'])
addpath([fileparts(s),'/rvctools'])
startup_rvc;

%addpath([fileparts(s),'/traj_gen'])
addpath([pth1,'cmu_quad_matlab/dry/src/geometry_utils'])
addpath([pth1,'cmu_quad_matlab/dry/src/quadrotor_model'])
addpath([pth1,'cmu_quad_matlab/dry/install_isolated/share/ipc_bridge/matlab'])
addpath([pth1,'cmu_quad_matlab/wet/build/lib']);