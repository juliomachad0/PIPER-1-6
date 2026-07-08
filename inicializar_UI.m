%% Reset all variables, functions, figures and parameters 
close all; clear all;
restoredefaultpath; rehash toolboxcache; close all hidden;
clear classes; clear functions; clearvars -global; clc;

%% Adding essencial paths
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'guiagem'));


%% Setting up
% range_time_without_gps: if [0 0], GPS considered during all trajectory
% [ti tf]: during ti and tf (ti <= t <= tf) GPS and yaw will not be used
% for correction
% use [a b; c d; e f] for multiples time ranges
range_time_without_correction = [0 0];
assignin('base','range_time_without_correction',range_time_without_correction);

%% Initiating simulation
inicializar; % Start parameters, models, functions and XPlane connection
gui_waypoints; % initiate UI to choose waypoints. 
