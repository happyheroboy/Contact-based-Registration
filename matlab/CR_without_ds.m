
% The method without dichotomic search
reset_toolbox;
clc;
close all

% radius of ball tip
radius = 21; 
isLeft = 1;
% load model
[original_points,original_faces,newPoints,newFace]=importModel("testPart.stl");
% [original_points,original_faces,newPoints,newFace]=importModel("right_ACM.stl");
%%
%rotate the model   
%角度：外展角和前倾角 two angles: anteversion and abduction
angle_alpha = deg2rad(90);
angle_gammar = deg2rad(0);
R_XYZ = angle2vector(angle_alpha,angle_gammar,isLeft);
loc = round(original_points*R_XYZ,4);

figure();
pcshow(pointCloud(loc));
view(3);
%%
%intial point drop
start_pts = [loc(:,1) loc(:,2) loc(:,3)+2.5*radius];
tic;
bottomPoint = [];
contactPoint = [];
for pt_num = 1:size(start_pts,1)
    fprintf('The is %2.1f th samplePoints\n',pt_num)
    initial_point = start_pts(pt_num,:);
    [index,~] = nbselect(loc,initial_point,'Z',radius);% 用Kd-tree采集球所"笼罩点云" kd-tree filte
    inPoints = loc(index{1},:);
    [bottomPoint1,contactPoint1] = downIteration(initial_point,inPoints,radius);
    bottomPoint = [bottomPoint;bottomPoint1];
    contactPoint = [contactPoint;contactPoint1];
end
toc;

keyPoint = bottomPoint(find(bottomPoint(:,3)==min(bottomPoint(:,3))),:);
keyPoint_contact = contactPoint(find(contactPoint(:,3)==min(contactPoint(:,3))),:);
trans_keyPoint = keyPoint*R_XYZ';
trans_keyPoint_contact = keyPoint_contact*R_XYZ'; 

figure()
pcshow(pointCloud(original_points));
hold on
% plot3(trans_keyPoint_contact(1,1),trans_keyPoint_contact(1,2),trans_keyPoint_contact(1,3),'*r')
plot3(trans_keyPoint(1,1),trans_keyPoint(1,2),trans_keyPoint(1,3),'*r')

[x,y,z] = sphere_test(trans_keyPoint',radius);
h = mesh(x,y,z);
h.FaceAlpha = 0.3;

function [bottomPoint,contactPoint] = downIteration(initial_point,data,radius)
%input data：
%initial_point 球心起始位置
%data 球所笼罩点云位置（圆柱体区域）
%radius 半径
%output data：
%bottomPoint 接触髋臼时，最低球心位置
%contactPoint 接触髋臼时，接触点位置

distances = pdist2(initial_point,data)';
z_dis = repmat(initial_point(:,3),size(data,1),1) - data(:,3);
up_dis = z_dis - (radius^2-distances.^2 + z_dis.^2).^0.5;
next_Point = [repmat(initial_point(:,1),size(data,1),1) repmat(initial_point(:,2),size(data,1),1)...
    repmat(initial_point(:,3),size(data,1),1)-up_dis];
index = find(up_dis==min(up_dis));
if size(index,1)~=1  %可能会出现多个相同落点
    bottomPoint = next_Point(index(1),:);
    contactPoint = data(index(1),:);
else
    bottomPoint = next_Point(index,:);
    contactPoint = data(index,:);
end

end

