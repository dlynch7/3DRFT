%% init_params.m
%
% Usage:
%
% Description:
%
% Inputs:
%
% Outputs:
%

function params = init_params(stlname)
%% 
ini_TR = stlread(['../../STL/',stlname,'.stl']);
% params.body.geom.scale_factor = 1e-3; % mm --> m
params.body.geom.scale_factor = 1e0; % mm --> m
params.body.geom.TR = triangulation(ini_TR.ConnectivityList,...
                                    params.body.geom.scale_factor*ini_TR.Points);

% compute the area of each triangle face:
params.body.geom.areas = NaN(1,size(params.body.geom.TR,1)); % preallocate
for s = 1:size(params.body.geom.TR,1)
    % vertices of current triangle element:
    vert_1 = params.body.geom.TR.Points(params.body.geom.TR.ConnectivityList(s,1),:);
    vert_2 = params.body.geom.TR.Points(params.body.geom.TR.ConnectivityList(s,2),:);
    vert_3 = params.body.geom.TR.Points(params.body.geom.TR.ConnectivityList(s,3),:);

    % area of triangle element:
    params.body.geom.areas(s) = norm(cross(vert_2-vert_1,vert_3-vert_1))/2;
end

%% General visualization parameters
params.viz.margin_scale = 2;
params.viz.unit_vector_arrow_scale = 0.1;
params.viz.frame_downsample = 1;
params.viz.GRF_vector_arrow_scale = 1;

%% Parameters related to the ground:
% RFT parameters:
coeffs = load('RFT_coeffs.mat');
params.terrain.rft.cubic_coefficients = [coeffs.coeffs(1:20)';
                                         coeffs.coeffs(21:40)';
                                         coeffs.coeffs(41:60)'];

params.terrain.scale_factor = 0.33e5;
params.terrain.surf_fric_coef = 1;

% impact plane geometry parameters:
params.terrain.geom.unit_normal = [0;0;1];
params.terrain.geom.point = zeros(3,1);

% plane visualization parameters:
params.terrain.viz.face.color = 0.5*[0,1,0];
params.terrain.viz.face.alpha = 0.25;
params.terrain.viz.edge.color = 'none';
params.terrain.viz.edge.alpha = 1;

end