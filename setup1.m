
HW = 1; % half width of the object
HL = 1; % half length of the object
HH = 1; % half height of the object

posrange = [-0.5,-0.5,-0.5,0.5,0.5,0.5];

ws = [-0.5,-0.5,-1.5;0.5,0.5,1.5];
ws1 = ws + [HW,0,0];
ws2 = ws + [-HW,0,0];
ws3 = ws + [0,HW,0];
ws4 = ws + [0,-HW,0];
workspace = [ws1(:);ws2(:);ws3(:);ws4(:)]';

nd = 4 + 1;
normals = [0,1,0;-1,0,0;0,-1,0;1,0,0;0,0,1;0,0,-1];
dim = [HL, HW, HH];
surface = [];
for k = 1:6
    n = normals(k,:);
    idx = find(n==0);
    nidx = find(n~=0);
    dd = dim(n==0);
    [XX,YY] = meshgrid(dd(1)/nd:dd(1)/nd:(dd(1)-dd(1)/nd), dd(2)/nd:dd(2)/nd:(dd(2)-dd(2)/nd));
    DD = zeros(numel(XX),6);
    DD(:,idx) = [XX(:),YY(:)];
    DD(:,nidx) = -n(nidx)*dim(nidx);
    DD(:,4:6) = repmat(n,numel(XX),1);
    surface = [surface; DD];
end

start_finger = [7,23,39,58];

start_obj = [0,0,0,1,0,0,0];
goal_obj = [0,0,0,0,1,0,0];
plannerid = 0;