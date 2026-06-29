%% Remove generated Simulink build artifacts (if present)
% this files can modify the gains of autopilot, which leads to errors
rootDir = fileparts(mfilename('fullpath'));

% File to remove
slxFile = fullfile(rootDir, 'NL_guidance.slxc');
if exist(slxFile, 'file') == 2
    try
        delete(slxFile);
        fprintf('Deleted file: %s\n', slxFile);
    catch ME
        warning('Could not delete file %s: %s', slxFile, ME.message);
    end
else
    fprintf('File not found (skipping): %s\n', slxFile);
end

% Folder to remove (recursively)
slprjFolder = fullfile(rootDir, 'slprj');
if exist(slprjFolder, 'dir') == 7
    try
        rmdir(slprjFolder, 's'); % 's' removes directory and contents
        fprintf('Removed folder: %s\n', slprjFolder);
    catch ME
        warning('Could not remove folder %s: %s', slprjFolder, ME.message);
    end
else
    fprintf('Folder not found (skipping): %s\n', slprjFolder);
end
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