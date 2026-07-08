%% Reset all variables, functions, figures and parameters 
close all; clear all;
restoredefaultpath; rehash toolboxcache; close all hidden;
clear classes; clear functions; clearvars -global; clc;

%% Adding essencial paths
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'guiagem'));


%% Setting up
%% range_time_without_gps: if [0 0], GPS considered during all trajectory
% [ti tf]: during ti and tf (ti <= t <= tf) GPS and yaw will not be used
% for correction
% use [a b; c d; e f] for multiples time ranges
range_time_without_correction = [0 0];
assignin('base','range_time_without_correction',range_time_without_correction);
%% Telemetry
% activate_telemetry:
%   true  -> imprime telemetria no Command Window
%   false -> desativa telemetria
%
% consider_FK_in_telemetry:
%   true  -> imprime também velocidade/euler do filtro/estimador
%   false -> imprime apenas dados do XPlane/comandos
%
% ins_telemetry_time_interval:
%   0   -> imprime sempre que a função for chamada
%   > 0 -> imprime a cada X segundos de simulação

activate_telemetry = true;
consider_FK_in_telemetry = false;
ins_telemetry_time_interval = 1;

assignin('base','activate_telemetry', activate_telemetry);
assignin('base','consider_FK_in_telemetry', consider_FK_in_telemetry);
assignin('base','ins_telemetry_time_interval', ins_telemetry_time_interval);
%% Initiating simulation
inicializar; % Start parameters, models, functions and XPlane connection
gui_waypoints; % initiate UI to choose waypoints. 
%% PONTOS UTILIZADOS NO TESTE
% XN   | YE
%   0  | 0
% 300  | 0
% 500  | 100
% 500  | 400
% 300  | 500
%   0  | 500
% altitude constante a 100 m
% velocidade constante a 15 m/s
%