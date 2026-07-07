addpath(fullfile(rootDir, 'navigation', 'sensors'));
addpath(fullfile(rootDir, 'navigation', 'plots'));
%% Adding sensors paths
addpath(fullfile(rootDir, 'navigation', 'sensors','ICM20689'));
addpath(fullfile(rootDir, 'navigation', 'sensors','NEO8M'));
ins_init_sensors(); % load variables para o modelo e os dados dos sensores


%% ========== DBN / Inertial Navigation Initialization ==========

rootDir = fileparts(fileparts(mfilename('fullpath')));

addpath(fullfile(rootDir, 'navigation', 'DBN'));

% Placeholder. O DBN_params real será criado por init_DBN,
% chamado dentro de ins_initial_state_xplane.m após reposicionar o X-Plane.
DBN_params = struct();
DBN_params.initialized = false;

assignin('base', 'DBN_params', DBN_params);

disp('--- DBN carregado no path. A inicialização real ocorrerá via init_DBN após posicionar o X-Plane. ---');