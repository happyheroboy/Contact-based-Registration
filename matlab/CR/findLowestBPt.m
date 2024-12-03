function [trans_keyPoint,trans_keyPoint_contact] = findLowestBPt(loc,R_XYZ,radius)
% �������������
% ��������---Ԥ��--�����
barycenter = mean(loc);
%��ʼ�����
Gpoint = [barycenter(:,1) barycenter(:,2) barycenter(:,3)+2.5*radius];

temp_initialPts = Gpoint;
all_bottomPoint = [];
all_contactPoint = [];

edgeLength = 8;
tic;
sample_num = 1;
record_bottomPt = [0,0,0];
while sample_num <= 20
    
    fprintf('The is %2.1f th samplePoints\n',sample_num);
    samplePoints = setSamplePts(temp_initialPts,edgeLength,4);
    edgeLength = edgeLength/2;
    
    % �õ���͵�λ��
    [L_bottomPoint,L_contactPoint] = computeBcPts(loc,samplePoints,radius);
    % [L_bottomPoint,L_contactPoint] = getTheBottomInAllSamples(loc,samplePoints,radius);
    % displayPointCloud(ptCloud,samplePoints,L_bottomPoint,L_contactPoint,radius,sample_num);
    
    all_bottomPoint = [all_bottomPoint;L_bottomPoint];
    all_contactPoint = [all_contactPoint;L_contactPoint];
    
    temp_initialPts = L_bottomPoint;
    temp_err = norm(temp_initialPts-record_bottomPt);
    record_bottomPt = L_bottomPoint;
    
    % ����ѭ������ֵ����
    if temp_err<1e-5
        break;
    end
    sample_num = sample_num + 1; 
    
end
toc;

%�ҵ���ͽӴ���λ�õ����ģ����ҽ�����ʾ����ת��ȥ���ž�ģ����
keyPoint = all_bottomPoint(find(all_bottomPoint(:,3)==min(all_bottomPoint(:,3))),:);
keyPoint_contact = all_contactPoint(find(all_bottomPoint(:,3)==min(all_bottomPoint(:,3))),:);
trans_keyPoint = keyPoint(1,:)*R_XYZ';
trans_keyPoint_contact = keyPoint_contact(1,:)*R_XYZ'; 

end
