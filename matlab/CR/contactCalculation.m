function [contactPts, spherePosition] = contactCalculation(trimeshInf,pointsCloudInf,radius,downPosition)
    
    % 策略：选取下落点附件500个点作为候选点，根据点找到相应的三角面片
    [indices,dists] = findNearestNeighbors(pointsCloudInf,downPosition,1000);
    
    %根据点找到相应的三角形mesh
    selected_pts = pointsCloudInf.Location(indices,:);
    index_XYZ = find(selected_pts(:,3)<=downPosition(3));
    selected_pts = selected_pts(index_XYZ);
    
    %找出与最近点相关的所有三角形mesh
    selceted_index = [];
    for i=1:size(selected_pts,1)
        index_1 = find(trimeshInf(:,1)==selected_pts(i,1));
        index_2 = find(trimeshInf(:,4)==selected_pts(i,1));
        index_3 = find(trimeshInf(:,7)==selected_pts(i,1));
        index = [index_1; index_2; index_3];
        selceted_index = [selceted_index;index];
    end
    selceted_index = unique(selceted_index);
    selected_triMeshs = trimeshInf(selceted_index,:);

    [contactPts_,spherePosition_,minDistance] = GetNextDownPosition(downPosition,selected_triMeshs,radius);
    
    contactPts = contactPts_;
    spherePosition = spherePosition_;
    
    %验证所得接触点是否为最深的
%     if abs(minDistance-radius)>0.1
%         [contactPts, spherePosition] = ...
%             contactCalculation(trimeshInf,pointsCloudInf,radius,spherePosition_);
%     else
%         contactPts = contactPts_;
%         spherePosition = spherePosition_;
%     end
    
end

function [contactPts,spherePosition,minDistance] = GetNextDownPosition(Gpoint,triM,radius)
    
    dis = [];  %距离记录
    pt = [];   %最近点记录
    for i = 1:size(triM,1)
    % fprintf('shift:%d/%d\r',i,size(triM,1));
        [dist,PP0] = pointTriangleDistance([triM(i,1:3); triM(i,4:6); triM(i,7:9)], Gpoint);%计算点与三角形的最近距离
        dis = [dis;dist];
        pt = [pt;PP0];
    end
    
    minDistance = min(dis); 
    index = find(dis==min(dis));
    all_pts = pt(index,:);
    %判断最近点是否为同一个点
    temp_pts = unique(all_pts,'rows');
    
    all_cPt = [];
    all_closestPt = [];
    for i = 1:size(temp_pts,1)
        closestPt = temp_pts(i,:); %最近点
        minDis = norm(closestPt-Gpoint);
        [spherePosition,contactPts] = computeContactPts(Gpoint,closestPt,radius,minDis);%接触点
        all_cPt = [all_cPt;spherePosition];
        all_closestPt = [all_closestPt;contactPts];
    end

    %多个最近点，可以得到多个最低点位置
    %有多个接触点，三个接触点才能使半球稳定在最低处
    if size(spherePosition,1)>1
        [spherePosition,ia,~] = unique(all_cPt,'rows');%是否会有
        temp_contactPts = all_closestPt(ia,:);
        index_ = find(spherePosition(:,3)==min(spherePosition(:,3)));
        spherePosition = spherePosition(index_,:);
        contactPts = temp_contactPts(index_,:);
    else 
        contactPts = all_closestPt;
        spherePosition = all_cPt;
    end
    
end

function [spherePt,touchPt] = computeContactPts(Gpoint,closestPt,radius,minDis)
    %判断半球体下降多少，可以接触到最近点
    %所求为球心点，接触点记录即可

    if abs(minDis-radius)<=0.1
        spherePt = Gpoint;
        touchPt = closestPt;
    else
        z = Gpoint(:,3) - closestPt(:,3);
        up_dis = z - sqrt(radius^2-minDis^2 + z^2);
        if ~(isreal(up_dis))
           up_dis = z;
        end
        spherePt = [Gpoint(:,1) Gpoint(:,2) Gpoint(:,3) - up_dis];
        touchPt = closestPt;
    end
end