function [contactPts, spherePosition] = contactCalculation(trimeshInf,pointsCloudInf,radius,downPosition)
    
    % ���ԣ�ѡȡ����㸽��500������Ϊ��ѡ�㣬���ݵ��ҵ���Ӧ��������Ƭ
    [indices,dists] = findNearestNeighbors(pointsCloudInf,downPosition,1000);
    
    %���ݵ��ҵ���Ӧ��������mesh
    selected_pts = pointsCloudInf.Location(indices,:);
    index_XYZ = find(selected_pts(:,3)<=downPosition(3));
    selected_pts = selected_pts(index_XYZ);
    
    %�ҳ����������ص�����������mesh
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
    
    %��֤���ýӴ����Ƿ�Ϊ�����
%     if abs(minDistance-radius)>0.1
%         [contactPts, spherePosition] = ...
%             contactCalculation(trimeshInf,pointsCloudInf,radius,spherePosition_);
%     else
%         contactPts = contactPts_;
%         spherePosition = spherePosition_;
%     end
    
end

function [contactPts,spherePosition,minDistance] = GetNextDownPosition(Gpoint,triM,radius)
    
    dis = [];  %�����¼
    pt = [];   %������¼
    for i = 1:size(triM,1)
    % fprintf('shift:%d/%d\r',i,size(triM,1));
        [dist,PP0] = pointTriangleDistance([triM(i,1:3); triM(i,4:6); triM(i,7:9)], Gpoint);%������������ε��������
        dis = [dis;dist];
        pt = [pt;PP0];
    end
    
    minDistance = min(dis); 
    index = find(dis==min(dis));
    all_pts = pt(index,:);
    %�ж�������Ƿ�Ϊͬһ����
    temp_pts = unique(all_pts,'rows');
    
    all_cPt = [];
    all_closestPt = [];
    for i = 1:size(temp_pts,1)
        closestPt = temp_pts(i,:); %�����
        minDis = norm(closestPt-Gpoint);
        [spherePosition,contactPts] = computeContactPts(Gpoint,closestPt,radius,minDis);%�Ӵ���
        all_cPt = [all_cPt;spherePosition];
        all_closestPt = [all_closestPt;contactPts];
    end

    %�������㣬���Եõ������͵�λ��
    %�ж���Ӵ��㣬�����Ӵ������ʹ�����ȶ�����ʹ�
    if size(spherePosition,1)>1
        [spherePosition,ia,~] = unique(all_cPt,'rows');%�Ƿ����
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
    %�жϰ������½����٣����ԽӴ��������
    %����Ϊ���ĵ㣬�Ӵ����¼����

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