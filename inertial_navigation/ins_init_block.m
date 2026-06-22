addpath(fullfile(rootDir, 'inertial_navigation', 'Xplane_Interface'));
addpath(fullfile(rootDir, 'inertial_navigation', 'sensors'));
addpath(fullfile(rootDir, 'inertial_navigation', 'plots'));

ins_init_sensors(); % load variables para o modelo e os dados dos sensores
ins_init_xplane; % inicia e configura conexão com Xplane