
clc
close all

%% 预处理

TR = stlread('D:\Matlab2019a\bin\XXZ_Matlab\paper_exp\sec_paper\kuanjiu_full.stl');

%显示髋臼
figure();
patch('Faces',TR.ConnectivityList,'Vertices',TR.Points,'FaceColor','w');
view(3);

%% virtual point cloud
virtual_points = TR.Points;

data_sf = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\5-22\surface_matching_pinpoint.txt");
r_acm_points = []; 
for i=1:(size(data_sf,1)/30)
    temp = data_sf(((i-1)*30+1):i*30,:);
    temp = rmoutliers(temp);
    r_acm_points= [r_acm_points;mean(temp)];
end

% load real_points
data_pp = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\5-22\paired_point_pinpoint.txt");
Real_values = []; 
for i=1:(size(data_pp,1)/30)
    temp = data_pp(((i-1)*30+1):i*30,:);
    temp = rmoutliers(temp);
    Real_values= [Real_values;mean(temp)];
end

%%  测试点
data_testLoad = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\5-22\targetPts.txt");

real_test_values = []; 
for i=1:(size(data_testLoad,1)/30)
    temp1 = data_testLoad(((i-1)*30+1):i*30,:);
    temp1 = rmoutliers(temp1);
    real_test_values= [real_test_values;mean(temp1)];
end

virtual_teset_values = load("virtual_AC_target_points.txt");
%%  virtual points
virtual_values = [
-118.1970 	-152.0204 	-265.6172
120.8548 	-153.8728 	-274.9739
14.7953 	-162.6469 	-347.0275 	
-15.3839 	-163.0026 	-344.5618 
65.8072 	 -94.2948 	-328.8401 ];

%%  computer RT
[R_r2v,t_r2v] = CalculateRTMatrix(Real_values,virtual_values);

%% real points

trans_r_points = (real_test_values*R_r2v + repmat(t_r2v,size(real_test_values,1),1));
coarse_acm_points = (r_acm_points*R_r2v + repmat(t_r2v,size(r_acm_points,1),1));

figure
scatter3(virtual_points(:,1),virtual_points(:,2),virtual_points(:,3),'green','filled');
hold on
scatter3(coarse_acm_points(:,1),coarse_acm_points(:,2),coarse_acm_points(:,3),'blue','filled');


%% ICP 精配
[R_final,t_final] = fuc_icp(coarse_acm_points(1:40,:),virtual_points);
% f_test_point_1 = (coarse_acm_points*R_final + repmat(t_final',size(coarse_acm_points,1),1));
% final_r_points = (trans_r_points*R_final + repmat(t_final',size(trans_r_points,1),1));

f_test_point_1 = (R_final*coarse_acm_points' + t_final)';
final_r_points = (R_final*trans_r_points' + t_final)';

% rmse_2 = sqrt(mean(sum((final_r_points' - v_initial_points').^2, 2)));
% err = [rmse_1;rmse_2];
dis_VR2 = [];
% TRE
for i=1:13
    temp_dis = norm(final_r_points(i,:)-virtual_teset_values(i,:));
    dis_VR2 = [dis_VR2; temp_dis];
end

sum_d = sum(dis_VR2.^2);
TRE_4 = sqrt(sum_d/size(real_test_values,1)); 
sum_d1 = sum(dis_VR2(1:12,:).^2);
TRE_5 = sqrt(sum_d1/12); 

figure
patch('Faces',TR.ConnectivityList,'Vertices',TR.Points,'FaceColor','w');
hold on
scatter3(f_test_point_1(:,1),f_test_point_1(:,2),f_test_point_1(:,3),'red','filled');

%%
% 以接触式方法为粗配准，ICP作为精配准

coarse_acm_points_1 = (R*r_acm_points'-repmat(t_r2v_11,1,size(r_acm_points,1)))';
tran_Pts = (R*real_test_values'-repmat(t_r2v_11,1,size(real_test_values,1)))';

% fine registration
[R_final_1,t_final_1] = fuc_icp(coarse_acm_points_1,virtual_points);

f_test_point_1 = (R_final_1*coarse_acm_points_1' + t_final_1)';
final_r_points = (R_final_1*tran_Pts' + t_final_1)';

% rmse_2 = sqrt(mean(sum((final_r_points' - v_initial_points').^2, 2)));
% err = [rmse_1;rmse_2];
dis_VR3 = [];
% TRE
for i=1:13
    temp_dis = norm(final_r_points(i,:)-virtual_teset_values(i,:));
    dis_VR3 = [dis_VR3; temp_dis];
end

sum_d = sum(dis_VR3.^2);
TRE_6 = sqrt(sum_d/size(real_test_values,1)); 
sum_d1 = sum(dis_VR3(1:12,:).^2);
TRE_7 = sqrt(sum_d1/12); 
