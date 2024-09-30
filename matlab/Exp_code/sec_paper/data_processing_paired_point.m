
clc
close all

% load real_points

data_pp = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\5-22\paired_point_pinpoint.txt");

Real_values = []; 
for i=1:(size(data_pp,1)/30)
    temp = data_pp(((i-1)*30+1):i*30,:);
    temp = rmoutliers(temp);
    Real_values= [Real_values;mean(temp)];
end

tr_points = Real_values;
Real_values = [Real_values(:,1),Real_values(:,2),Real_values(:,3)];

%%  virtual points

virtual_values = [
-118.1970 	-152.0204 	-265.6172
120.8548 	-153.8728 	-274.9739
14.7953 	-162.6469 	-347.0275 	
-15.3839 	-163.0026 	-344.5618 
65.8072 	 -94.2948 	-328.8401 ];

% virtual_values = [65.8072 	 -94.2948 	-328.8401 	
%                   120.8548 	-153.8728 	-274.9739 	
%                   14.7953 	-162.6469 	-347.0275 	
%                   -15.3839 	-163.0026 	-344.5618 	
%                  -118.1970 	-152.0204 	-265.6172];

% virtual_values = [65.8072 	 -94.2948 	-328.8401 	
%                   123.792313, -155.112869, -270.958984 	
%                   14.7953 	-162.6469 	-347.0275 	
%                   -15.3839 	-163.0026 	-344.5618 	
%                  -118.510033, -153.187561, -262.682251];

%%  computer RT

[R_r2v,t_r2v] = CalculateRTMatrix(Real_values,virtual_values);

%% load test points

data_testLoad = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\5-22\targetPts.txt");

real_test_values = []; 
for i=1:(size(data_testLoad,1)/30)
    temp1 = data_testLoad(((i-1)*30+1):i*30,:);
    temp1 = rmoutliers(temp1);
    real_test_values= [real_test_values;mean(temp1)];
end

virtual_teset_values = load("virtual_AC_target_points.txt");

% vr_pts = [real_test_values(1,:);real_test_values(6,:);real_test_values(16,:);real_test_values(21,:);real_test_values(25,:)];
% vv_pts = [virtual_teset_values(1,:);virtual_teset_values(6,:);virtual_teset_values(16,:);virtual_teset_values(21,:);virtual_teset_values(25,:)];
% [R_r2v,t_r2v] = CalculateRTMatrix(vr_pts(1:5,:),vv_pts(1:5,:));

trans_rtest_values = real_test_values*R_r2v + repmat(t_r2v,size(real_test_values,1),1);

dis_VR = [];
% TRE
for i=1:13
    temp_dis = norm(trans_rtest_values(i,:)-virtual_teset_values(i,:));
    dis_VR = [dis_VR; temp_dis];
end

sum_d = sum(dis_VR.^2);
TRE = sqrt(sum_d/size(real_test_values,1)); 

sum_d1 = sum(dis_VR(1:12,:).^2);
TRE_1 = sqrt(sum_d1/12); 
