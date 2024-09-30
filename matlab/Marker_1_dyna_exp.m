
reset_toolbox;
clc
close all
load 10x10_for_3x3_6x2_10x1.mat
load('calibrationSession_8-26.mat')
load('square_maker_model.mat')
stereoParams = calibrationSession.CameraParameters; 

Model = point3D_XYZ;
imageNum = 25;
tip_points = [];
total_err = [];

for b=1:imageNum
   
    ml=im2double(imread(['C:\Users\Lenovo\Desktop\MJ_re-exp\Marker1\dyn_exp2\0\',int2str(b),'.bmp']));
    mr=im2double(imread(['C:\Users\Lenovo\Desktop\MJ_re-exp\Marker1\dyn_exp2\1\',int2str(b),'.bmp']));

    ml = undistortImage(ml,stereoParams.CameraParameters1);
    mr = undistortImage(mr,stereoParams.CameraParameters2); 
    %% 
    %% 特征点
    expectN = 2*(size(sta,1)+1)*(size(sta,2)+1);
    [ptListl,edgel] = read_marker(ml,sta,5,expectN,3);    
    [ptListr,edger] = read_marker(mr,sta,5,expectN,3);

    %%
    ptListl = [ ptListl(:,2) ptListl(:,1) ptListl(:,3)];
    ptListr = [ ptListr(:,2) ptListr(:,1) ptListr(:,3)];

    %% 三角测量

    [c,ia,ib]=intersect(ptListl(:,3),ptListr(:,3));

    %%计算第一对匹配点的三维值
    [point3D,err]=triangulate(ptListl(ia,1:2),ptListr(ib,1:2),stereoParams);
    point3D=[point3D ptListl(ia,3)];
    total_err = [total_err;err];
    
    %% 跟踪

    [c,ia1,ib1] = intersect(Model(:,4),point3D(:,4));
%     [r_mat,t_mat]  = CalculateRTMatrix(pre3Did(ia,1:3),cur3Did(ib,1:3));

    [r_1,t_1]  = CalculateRTMatrix(Model(ia1,1:3),point3D(ib1,1:3));
    trans_model = Model(:,1:3)*r_1 + repmat(t_1,123,1);
    tip_points = [tip_points; trans_model(123,:)];

end

%%
temp_da = tip_points;
% 创建X轴和Y轴的坐标向量
x = 0:25:100; % 0, 25, 50, 75, 100
y = 0:25:100; % 0, 25, 50, 75, 100

% 使用meshgrid函数创建网格
[X, Y] = meshgrid(x, y);
Z=repmat(100,5);

sim_data = [X(:) Y(:) Z(:)];

%%
[R,t] = CalculateRTMatrix(sim_data,temp_da(1:25,:));

tran_data = sim_data*R + repmat(t,25,1);
figure()
scatter3(temp_da(:,1),temp_da(:,2),temp_da(:,3),120,'red','filled');
hold on
scatter3(tran_data(:,1),tran_data(:,2),tran_data(:,3),120,'blue','filled')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 20)
set(gca, 'Box', 'off', ...                                         % 边框
         'LineWidth', 1.5)

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

%%
% 计算FRE
sum_err = 0;

for i=1:size(tran_data,1)
    temp_err = norm(tran_data(i,:)-temp_da(i,:));
    sum_err = temp_err + sum_err;
end

fre = sum_err/25;
