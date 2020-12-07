clear; clc;

setup1;

workspace_T = workspace';
surface_T = surface';
[object_path, finger_path, pathlength, treesize] = ...
    planner(posrange, workspace_T(:) ,surface_T(:), start_obj, start_finger, goal_obj, plannerid,max_samples,rrtstar_radius);

object_path(1,:) = start_obj;
%plot_animation(HL, HW, HH, workspace, surface, object_path, finger_path);
