function [L_bottomPoint,L_contactPoint,op_num] = computeBcPts(orginal_data,samplePts,radius)
%���������λ�ã�����ÿ����������ȽӴ��ž�ʱ������λ��
%input data��
%orginal_data ԭʼ�žʵ���
%samplePts ������λ��
%radius ��ͷ�뾶

%output data��
%bottomPoint ����λ��
%contactPoint �Ӵ���λ��
%��һ��

sp_center = samplePts(13,:);%�����������ĵ㣨��ʼ�㣩
gridPts = samplePts;
n = [];
bottomPoint = [];
contactPoint = [];
while 1 %% �����Ǹ���ѭ����Ҫע����ֹ������ʲô
    
%     [index,~] = nbselect(orginal_data,sp_center,'Z',radius);% ��Kd-tree�ɼ����������ֵ��ơ�
%     inPoints = orginal_data(index{1},:); %��״����
%     tic
    inPoints = getCylinder_cloud(orginal_data,sp_center,radius);%��״����
    if isempty(inPoints)
        break;
    end
%     toc
    [bottomPoint1,contactPoint1] = downIteration(sp_center,inPoints,radius);%��Ӵ���
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
%samplePoints ���в�����
%original_centerPoint ��ʼ�����
%contactPoint �Ӵ���

%output
%new_samples �Ż���Ĳ�����
%new_centerPoints �Ż������ʼ�����
if size(samplePoints,1)==1
    new_samples = [];
    new_centerPoints = [];
else
    %ɸѡԲ�İ뾶
    r_dis = norm(contactPoint(:,1:2)-original_centerPoint(:,1:2));
    %���е���Բ�ģ��Ӵ��㣩�ľ���
    all_dis = pdist2(contactPoint(:,1:2),samplePoints(:,1:2))';
    
    saved_index = find(round(all_dis,4)>round(r_dis,4));
    if isempty(saved_index) %ʣ������㱻ȫ������
        new_samples = [];
        new_centerPoints = [];
    else %ʣ�������δ��ȫ������
        new_samples = samplePoints(saved_index,:);
        dis2center = pdist2(original_centerPoint(:,1:2),new_samples(:,1:2))';
        newCenter_index = find(dis2center==min(dis2center));
        new_centerPoints = new_samples(newCenter_index(1),:);
    end
end
end

function [bottomPoint,contactPoint] = downIteration(initial_point,data,radius)
%input data��
%initial_point ������ʼλ��
%data �������ֵ���λ�ã�Բ��������
%radius �뾶
%output data��
%bottomPoint �Ӵ��ž�ʱ���������λ��
%contactPoint �Ӵ��ž�ʱ���Ӵ���λ��

distances = pdist2(initial_point,data)';
z_dis = repmat(initial_point(:,3),size(data,1),1) - data(:,3);
up_dis = z_dis - (radius^2-distances.^2 + z_dis.^2).^0.5;

next_Point = [repmat(initial_point(:,1),size(data,1),1) repmat(initial_point(:,2),size(data,1),1)...
    repmat(initial_point(:,3),size(data,1),1)-up_dis];
index = find(up_dis==min(up_dis));
if size(index,1)~=1  %���ܻ���ֶ����ͬ���
    bottomPoint = next_Point(index(1),:);
    contactPoint = data(index(1),:);
else
    bottomPoint = next_Point(index,:);
    contactPoint = data(index,:);
end

end