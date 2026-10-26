%% main.m
%
% Description:
%   Program entry point. Calls various functions.
%
% Dependencies:
%   Modern Robotics
%
% Inputs:
%   none
%
% Outputs:
%   none

function main

clear;
close all;
clc;

init_env;

%% Initialize parameters
% stl_name = 'shapes/1_sphere_radius_0p05m';
stl_name = 'shapes/8_bunny_stl';
params = init_params(stl_name);
params.viz.GRF_vector_arrow_scale = 5;

%% Initialize configuration
% initialize orientation:
ang_x = 0;
ang_y = 0;
ang_z = 0;
Rx = [1 0 0;
      0 cos(ang_x) -sin(ang_x);
      0 sin(ang_x)  cos(ang_x)];
Ry = [cos(ang_y) 0 sin(ang_y);
      0          1 0;
     -sin(ang_y) 0 cos(ang_y)];
Rz = [cos(ang_z) -sin(ang_z) 0;
     sin(ang_z)  cos(ang_z) 0;
     0           0          1];
Rwb = Rz*Ry*Rx;

% initialize position so the centroid of the object is below ground:
centroid_xyz = centroid3(params.body.geom.TR);
pwb = -centroid_xyz; % center object at the origin
pwb(3) = pwb(3) - 1e-1; % place the object below ground:
Twb = RpToTrans(Rwb,pwb);

%% Prescribe twist
% The prescribed body-frame twist will be a constant screw motion:
% simultaneous rotation about and translation along a world-frame axis.
omg_b_x = 0;
omg_b_y = 0;
omg_b_z = 0;
v_b_x = 0;
v_b_y = 0;
v_b_z = -1;

Vb = [omg_b_x;
     omg_b_y;
     omg_b_z;
     v_b_x;
     v_b_y;
     v_b_z];
 
%% Compute ground reaction wrench in world frame
[Fb,~,~,local_force,C] = ...
    vec_compute_wrench_3DRFT(Twb,Vb,params);

fprintf('Mx = %.2E\n',Fb(1))
fprintf('My = %.2E\n',Fb(2))
fprintf('Mz = %.2E\n',Fb(3))
fprintf('Fx = %.2E\n',Fb(4));
fprintf('Fy = %.2E\n',Fb(5));
fprintf('Fz = %.2E\n',Fb(6));

%% Visualize
points = [params.body.geom.TR.Points,...
          ones(size(params.body.geom.TR.Points,1),1)]*transpose(Twb)*[eye(3);zeros(1,3)];
TR = triangulation(params.body.geom.TR.ConnectivityList,points);

colormap turbo
trisurf(TR,...
        'EdgeColor','k','EdgeAlpha',0.25,...
        'FaceAlpha',0.1,'FaceColor',[0.5 0.5 0.5]);
hold on
quiver3(C(1,:),...
        C(2,:),...
        C(3,:),...
        local_force(1,:),...
        local_force(2,:),...
        local_force(3,:),...
        params.viz.GRF_vector_arrow_scale,...
        'color','r','LineWidth',2);
hold on
    
xmin = min(C(1,:));
xmax = max(C(1,:));
ymin = min(C(2,:));
ymax = max(C(2,:));
zmin = min(C(3,:));
zmax = max(C(3,:));

surf([xmin xmax],[ymin ymax],zeros(2,2),...
    'FaceColor',params.terrain.viz.face.color,...
    'FaceAlpha',params.terrain.viz.face.alpha,...
    'EdgeColor',params.terrain.viz.edge.color,...
    'EdgeAlpha',params.terrain.viz.edge.alpha);
    
xlabel('$x$')
ylabel('$y$')
zlabel('$z$')
axis([xmin xmax ymin ymax zmin zmax]);
view([45,30]);
axis equal
axis vis3d

% legend('intruder','surface normal','local velocity','local force','Location','NorthOutside')

end