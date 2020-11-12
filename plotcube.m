function h = plotcube(origin,X,Y,Z,R,cdata,alpha)
% PLOTCUBE plots a cube with dimension of X, Y, Z.
%
% INPUTS:
% origin = set origin point for the cube in the form of [x,y,z].
% X      = cube length along x direction.
% Y      = cube length along y direction.
% Z      = cube length along z direction.
% R      = rotation matrix of the object
% cdata  = numeric value used to determine the color of the cube when
%          colormap and colorbar are used.
% OUPUTS:
% Plot a figure in the form of cubics.
%
%
% Originally by: Jialun Liu
%                http://jialunliu.com/how-to-use-matlab-to-plot-3d-cubes/
% 
% Modified by:   Aleix Pinardell
%                to include rotation and CData
%


% ------------------------------Code Starts Here------------------------------ %
origin(1) = origin(1);
origin(2) = origin(2);
origin(3) = origin(3);
% Define the vertexes of the unit cubic
ver = [1 1 0;
    0 1 0;
    0 1 1;
    1 1 1;
    0 0 1;
    1 0 1;
    1 0 0;
    0 0 0]-[0.5,0.5,0.5];

%  Define the faces of the unit cubic
fac = [1 2 3 4;
    4 3 5 6;
    6 7 8 5;
    1 2 8 7;
    6 7 1 4;
    2 3 5 8];

% Rotate cube
cube = [ver(:,1)*X,ver(:,2)*Y,ver(:,3)*Z];
for i = 1:length(cube)
    cube(i,:) = (R*cube(i,:)')';
end

% Plot cube
cube = [cube(:,1)+origin(1),cube(:,2)+origin(2),cube(:,3)+origin(3)];
h = patch('Faces',fac,'Vertices',cube,'FaceVertexCData',cdata,'FaceColor','flat','FaceAlpha',alpha, 'EdgeColor','none'); % or lines(6)

end
