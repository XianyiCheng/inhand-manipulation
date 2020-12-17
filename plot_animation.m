function plot_animation(HL, HW, HH, workspace, surface, object_path, finger_path)
figure;
waitforbuttonpress;
n_fingers = size(workspace,1)/2;
R = eye(3);
for i = 1:n_fingers
    ws_lb = workspace((i-1)*2+1,:);
    ws_ub = workspace(i*2,:);
    origin = (ws_lb + ws_ub)/2;
    l = abs(ws_ub - ws_lb);
    plotcube(origin,l(1),l(2),l(3),R,autumn(1),0.1);
    hold on;
end

markersize = 60;
axis equal;
grid on;
xlim([-HL*3 HL*3])
ylim([-HW*3 HW*3])
zlim([-HH*3 HH*3])
xlabel('X','FontSize',14);
ylabel('Y','FontSize',14)
zlabel('Z','FontSize',14)
view(120,30);

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
