%% Reset all variables, figures and parameters 
close all; clear all; clc;
%% Adding paths - initiating simulation 
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'guiagem')); % guidance
%% Flight Dynamics - sfunction_piper, aerodynamics, dyn_rigidbody, etc.
addpath(fullfile(rootDir, 'modelos', 'Não Linear'));
%% navigation block
addpath(fullfile(rootDir, 'navigation'));
addpath(fullfile(rootDir, 'navigation', 'sensors'));
addpath(fullfile(rootDir, 'navigation', 'sensors','ICM20689'));
addpath(fullfile(rootDir, 'navigation', 'FK'));
addpath(fullfile(rootDir, 'navigation', 'DBN'));
%% Plots
addpath(fullfile(rootDir, 'plots'));
%% XPlane Connection
addpath(fullfile(rootDir, 'xplane','Xplane_Interface','interface'));
addpath(fullfile(rootDir, 'xplane','XPlaneConnect-master','MATLAB'));
gui_waypoints;