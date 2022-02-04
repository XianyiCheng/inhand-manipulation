HL = 0.05; % half width of the object
HW = 1.0; % half length of the object
HH = 0.05; % half height of the object

posrange = [-0.2,-0.5,-0.2,0.2,0.5,0.2];


ws1 = [-0.2,-0.2,-0.15;...
       0.2,0.2,0.15];
ws2 = ws1 + [0, 0.2, 0];
ws3 = ws1 - [0, 0.2,0];
ws4 = [0,-0.4,-0.15; 0.3, 0.4,0.15]; % thumb

workspace = [ws1;ws2;ws3;ws4];

surface = [];
for dl = -HW:0.05:HW
    surface = [surface; HL, dl, 0, -1, 0, 0; -HL, dl, 0, 1, 0, 0];
end
surface = [-HL, mean(ws1(:,2)), 0, 1, 0, 0; ...
           -HL, mean(ws2(:,2)), 0, 1, 0, 0; ...
           -HL, mean(ws3(:,2)), 0, 1, 0, 0; ...
            HL, mean(ws4(:,2)), 0, -1, 0, 0; surface];
    

start_finger = [1,2,3,4];

% quaternion in matlab
% quaternion in eigen3: w,x,y,z
start_obj = [0,0,0, 1,0,0,0];
goal_obj = [0,0,0, 0,0,0,1];

plannerid = 0;
rrtstar_radius = 0.5;
 
max_samples = 1000;

