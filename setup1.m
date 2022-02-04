
HW = 1; % half width of the object
HL = 1; % half length of the object
HH = 1; % half height of the object

posrange = [-0.5,-0.5,-0.5,0.5,0.5,0.5];

ws_a = 0.6;
ws = [-ws_a,-ws_a,-1.5;ws_a,ws_a,1.5];
ws1 = ws + [HW,0,0];
ws2 = ws + [-HW,0,0];
ws3 = ws + [0,HW,0];
ws4 = ws + [0,-HW,0];
workspace = [ws1;ws2;ws3;ws4];

nd = 4 + 1;
normals = [0,1,0;-1,0,0;0,-1,0;1,0,0;0,0,1;0,0,-1];
dim = [HL, HW, HH];
surface = [];
for k = 1:6
    n = normals(k,:);
    idx = find(n==0);
    nidx = find(n~=0);
    dd = dim(n==0);
    [XX,YY] = meshgrid(-(dd(1)-dd(1)/(0.5*nd)):dd(1)/(0.5*nd):(dd(1)-dd(1)/(0.5*nd)),...
        -(dd(2)-dd(2)/(0.5*nd)):dd(2)/(0.5*nd):(dd(2)-dd(2)/(0.5*nd)));
    DD = zeros(numel(XX),6);
    DD(:,idx) = [XX(:),YY(:)];
    DD(:,nidx) = -n(nidx)*dim(nidx);
    DD(:,4:6) = repmat(n,numel(XX),1);
    surface = [surface; DD];
end

start_finger = [23, 54, 43, 7];

start_obj = [0,0,0,1,0,0,0];
%goal_obj = [0,0,0,rotm2quat(eul2rotm([0,pi,0])*eul2rotm([0,0,pi]))];
goal_obj = [0,0,0,0,1,0,0];
% goal_obj = [0,0,0,rotm2quat(axang2rotm([-pi/3,0,0])*axang2rotm([0,pi/2,0])*axang2rotm([0,0,pi/4]))];
plannerid = 0;
rrtstar_radius = 1.5;

max_samples = 1000;

