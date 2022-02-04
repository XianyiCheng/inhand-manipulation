function plot_animation_pen(HL, HW, HH, workspace, surface, object_path, finger_path)
figure;
waitforbuttonpress;
n_fingers = size(workspace,1)/2;
R = eye(3);
cdatas = [0,1,0.5; 0,0.5,1; 0.5,1,1; 0.9,0.1,0.1]
for i = 1:n_fingers
    ws_lb = workspace((i-1)*2+1,:);
    ws_ub = workspace(i*2,:);
    origin = (ws_lb + ws_ub)/2;
    l = abs(ws_ub - ws_lb);
    plotcube(origin,l(1),l(2),l(3),R,cdatas(i,:),0.2);
    hold on;
end

markersize = 60;

grid on;
xlabel('X','FontSize',14);
ylabel('Y','FontSize',14)
zlabel('Z','FontSize',14)
view(120,30);

axis equal;

xlim([-HW HW])
ylim([-HW HW])
zlim([-HW HW])

n = size(object_path,1);

for i = 1:n
    h = [];
    q = object_path(i,:);
    R = quat2rotm(q(4:7));
    h1 = plotcube(q(1:3),HL*2,HW*2,HH*2,R,parula(6),0.75);
    hold on;
    h = [h,h1];
    for j = 1:n_fingers
        pp = R*surface(finger_path(i,j),1:3)' + q(1:3)';
        hj = scatter3(pp(1),pp(2),pp(3),markersize,'filled','MarkerFaceColor','red','MarkerEdgeColor','k');
        hold on;
        h = [h,hj];
    end
    pause(0.5);
    if i ~= n
        for k = 1:numel(h)
            delete(h(k));
        end
    end
end
