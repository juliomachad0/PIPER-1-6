addpath(fullfile(rootDir, 'navigation', 'sensors'));
addpath(fullfile(rootDir, 'navigation', 'plots'));
%% Adding sensors paths
addpath(fullfile(rootDir, 'navigation', 'sensors','ICM20689'));

ins_init_sensors(); % load variables para o modelo e os dados dos sensores