% The method with dichotomic search
reset_toolbox;
clc;
close all

% radius of ball tip
radius = 21; 
%note�������ž�   ��չ�Ǻ�ǰ���в�һ��, ���б�һ��
isLeft = 1;

% %ģ�͵��� load model
[original_points,original_faces]=importModel("ac_part.stl");
% [original_points,original_faces,newPoints,newFace]=importModel("left_ACM.stl");
figure();
pcshow(pointCloud(original_points));

%%
%rotate the model   
%�Ƕȣ���չ�Ǻ�ǰ��� two angles: anteversion and abduction
angle_alpha = deg2rad(90); %ǰ���
angle_gammar = deg2rad(0);  
R_XYZ = angle2vector(angle_alpha,angle_gammar,isLeft);
loc = round(original_points*R_XYZ,4);

%��ʾ��ת����ž�
figure();
pcshow(pointCloud(loc));
view(3);

%%
% ������������� grid point
% ��������---Ԥ��--����� initial drop point
barycenter = mean(loc);
Gpoint = [barycenter(:,1) barycenter(:,2) barycenter(:,3)+2.5*radius];
% Gpoint = [-43.3 43.3 Gpoint(3)];
ptCloud = pointCloud(loc(:,1:3));
figure
pcshow(ptCloud);
hold on
plot3(Gpoint(1),Gpoint(2),Gpoint(3),'*r');

%%
temp_initialPts = Gpoint;
all_bottomPoint = [];
all_contactPoint = [];

edgeLength = 4;
tic;
sample_num = 1;
record_bottomPt = [0,0,0];
samPT=[];
while sample_num <= 20
    
    fprintf('The is %2.1f th samplePoints\n',sample_num);
    samplePoints = setSamplePts(temp_initialPts,edgeLength,8);
    samPT =[samPT;samplePoints];
    edgeLength = edgeLength/2;
    
    % �õ���͵�λ��
    [L_bottomPoint,L_contactPoint] = computeBcPts(loc,samplePoints,radius);
    % [L_bottomPoint,L_contactPoint] = getTheBottomInAllSamples(loc,samplePoints,radius);
    displayPointCloud(ptCloud,samplePoints,L_bottomPoint,L_contactPoint,radius,sample_num);
    
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
%     
end
toc;

%%
%�ҵ���ͽӴ���λ�õ����ģ����ҽ�����ʾ����ת��ȥ���ž�ģ����
keyPoint = all_bottomPoint(find(all_bottomPoint(:,3)==min(all_bottomPoint(:,3))),:);
keyPoint_contact = all_contactPoint(find(all_bottomPoint(:,3)==min(all_bottomPoint(:,3))),:);
trans_keyPoint = keyPoint*R_XYZ';
trans_keyPoint_contact = keyPoint_contact*R_XYZ'; 

figure()
pcshow(pointCloud(original_points));
hold on
plot3(trans_keyPoint(1,1),trans_keyPoint(1,2),trans_keyPoint(1,3),'*r')
[x,y,z] = sphere_test(trans_keyPoint',radius);
h = mesh(x,y,z);
h.FaceAlpha = 0.1;

function displayPointCloud(ptCloud,samplePoints,L_bottomPoint,L_contactPoint,radius,i)
figure
pcshow(ptCloud);
hold on;
plot3(samplePoints(:,1),samplePoints(:,2),samplePoints(:,3),'*','Color','g');
plot3(samplePoints(13,1),samplePoints(13,2),samplePoints(13,3),'*','Color','r');
hold on;
plot3(L_bottomPoint(:,1),L_bottomPoint(:,2),L_bottomPoint(:,3),'*','Color','r','MarkerSize',10);
plot3( L_contactPoint(:,1), L_contactPoint(:,2), L_contactPoint(:,3),'x','Color','r','MarkerSize',15);
[x,y,z] = sphere_test(L_bottomPoint',radius);
h = mesh(x,y,z);
h.FaceAlpha = 0.3;
end

