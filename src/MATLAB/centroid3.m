%% centroid3.m
%
% Description:
%   This function returns the coordinates [x,y,z] of a 3D polyhedron
%   represented by the triangulation TR.
%
% References:
%   Based on
%   https://www.mathworks.com/matlabcentral/fileexchange/63614-centroid-of-triangulated-polyhedron/
%
% Inputs:
%   TR: triangulation object containing a set of vertices and a
%       connectivity list
%
% Outputs:
%   centroid_xyz: centroid coordinates (x,y,z)

function centroid_xyz = centroid3(TR)

vertex = TR.Points;
faces = TR.ConnectivityList;

% find the area of closed polyhedron
vector1 = vertex(faces(:,2),:) - vertex(faces(:,1),:);
vector2 = vertex(faces(:,3),:) - vertex(faces(:,1),:);

triangAreasTmp = 0.5*cross(vector1,vector2);
triangAreas(:,1) = (triangAreasTmp(:,1).^2 + triangAreasTmp(:,2).^2 ...
               +  triangAreasTmp(:,3).^2).^(1/2); % 2-norm
totArea = sum(triangAreas); % total area

point1 = vertex(faces(:,1),:);
point2 = vertex(faces(:,2),:);
point3 = vertex(faces(:,3),:);

% center of each triangle:
centroidTriangles = (1/3).*(point1 + point2 + point3);

mg(:,1) = triangAreas(:,1).*centroidTriangles(:,1);
mg(:,2) = triangAreas(:,1).*centroidTriangles(:,2);
mg(:,3) = triangAreas(:,1).*centroidTriangles(:,3);

mg2 = [mg(:,1) mg(:,2) mg(:,3)];

centroid_xyz = sum(mg2)./totArea;
centroid_xyz = centroid_xyz';
end