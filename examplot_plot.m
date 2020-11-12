figure

markersize = 60;
scatter3([1,2,1],[1,0.4,-1],[1,-0.1,-1],markersize,'filled','MarkerFaceColor','red','MarkerEdgeColor','k')


plotcube([1 -1 -1],.2,.2,.2,0,0,0,autumn(1),0.1);

axis equal;

grid on;
xlim([-2 2])
ylim([-2 2])
zlim([-1 1])
xlabel('X','FontSize',14);
ylabel('Y','FontSize',14)
zlabel('Z','FontSize',14)
view(120,30);
h = plotcube([0 0 0],2,2,2,0,0,140,parula(6),0.75);
pause(0.1);
delete(h);
plotcube([0 0 0],2,2,2,0,0,100,parula(6),0.75);
%hold off;