function [L_bottomPoint,L_contactPoint,op_num] = computeBcPts(orginal_data,samplePts,radius)
%根据下落点位置，计算每个下落点最先接触髋臼时的球心位置
%input data：
%orginal_data 原始髋臼点云
%samplePts 采样点位置
%radius 球头半径

%output data：
%bottomPoint 球心位置
%contactPoint 接触点位置
%第一次

sp_center = samplePts(13,:);%采样网格中心点（起始点）
gridPts = samplePts;
n = [];
bottomPoint = [];
contactPoint = [];
while 1 %% 这里是个死循环，要注意终止条件是什么
    
%     [index,~] = nbselect(orginal_data,sp_center,'Z',radius);% 用Kd-tree采集球所“笼罩点云”
%     inPoints = orginal_data(index{1},:); %柱状点云
%     tic
    inPoints = getCylinder_cloud(orginal_data,sp_center,radius);%柱状点云
    if isempty(inPoints)
        break;
    end
%     toc
    [bottomPoint1,contactPoint1] = downIteration(sp_center,inPoints,radius);%求接触点
    [new_samples1,new_centerPoints1] = samplesUpdata(gridPts,sp_center,contactPoint1);
    temp_n = size(gridPts,1);
    n = [n;temp_n];
    
    bottomPoint = [bottomPoint;bottomPoint1];
    contactPoint = [contactPoint;contactPoint1];
    
    if isempty(new_samples1)
        break;
    end
    
    sp_center = new_centerPoints1;
    gridPts = new_samples1;
end

op_num = n;
index_x = find(bottomPoint(:,3)==min(bottomPoint(:,3)));
L_bottomPoint = bottomPoint(index_x,:);
L_contactPoint = contactPoint(index_x,:);

end

function [new_samples,new_centerPoints] = samplesUpdata(samplePoints,original_centerPoint,contactPoint)
%input
%samplePoints 所有采样点
%original_centerPoint 起始下落点
%contactPoint 接触点

%output
%new_samples 优化后的采样点
%new_centerPoints 优化后的起始下落点
if size(samplePoints,1)==1
    new_samples = [];
    new_centerPoints = [];
else
    %筛选圆的半径
    r_dis = norm(contactPoint(:,1:2)-original_centerPoint(:,1:2));
    %所有点与圆心（接触点）的距离
    all_dis = pdist2(contactPoint(:,1:2),samplePoints(:,1:2))';
    
    saved_index = find(round(all_dis,4)>round(r_dis,4));
    if isempty(saved_index) %剩余采样点被全部包括
        new_samples = [];
        new_centerPoints = [];
    else %剩余采样点未被全部包括
        new_samples = samplePoints(saved_index,:);
        dis2center = pdist2(original_centerPoint(:,1:2),new_samples(:,1:2))';
        newCenter_index = find(dis2center==min(dis2center));
        new_centerPoints = new_samples(newCenter_index(1),:);
    end
end
end

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