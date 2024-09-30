reset_toolbox;
close all;
load('calibrationSession_8-26.mat')
load('probe_model.mat')
stereoParams = calibrationSession.CameraParameters;

A = [];
calcu_values = [];
Model = point3D_XYZ;

for i = 1:25

    fprintf('第%d帧\n',i);
    fileName0 = ['.\Data\Marker-\dyna1\0\',int2str(i),'.bmp'];
    fileName1 = ['.\Data\Marker-\dyna1\1\',int2str(i),'.bmp'];

    img0 = im2double(imread(fileName0));
    img1 = im2double(imread(fileName1));
    
    img0 = undistortImage(img0,stereoParams.CameraParameters1);
    img1 = undistortImage(img1,stereoParams.CameraParameters2);

    [ptList0,~] = chess_detector(img0);
    [ptList1,~] = chess_detector(img1);

    ptList0 = [ptList0(:,2),ptList0(:,1)];
    ptList1 = [ptList1(:,2),ptList1(:,1)];

    ptList0 = sortrows(ptList0,2);
    ptList1 = sortrows(ptList1,2);

    %% 三角测量
    [point3D,err] = triangulate(ptList0,ptList1,stereoParams);

    A = [A;mean(err)];
    
    %% 跟踪
    [r_1,t_1]  = SVDICPxiugai(Model(1:3,1:3),point3D);
    trans_model = Model(:,1:3)*r_1 + repmat(t_1,4,1);

    calcu_values = [calcu_values;trans_model(4,:)];

end
%%

x = 0:25:100; % 0, 25, 50, 75, 100
y = 0:25:100; % 0, 25, 50, 75, 100

[X, Y] = meshgrid(x, y);
Z=repmat(100,5);

sim_data = [X(:) Y(:) Z(:)];

% 显示网格
figure()
scatter3(X(:), Y(:), Z(:), 'filled');

temp_da = calcu_values;
%%
[R,t] = SVDICPxiugai(sim_data,temp_da);

tran_data = sim_data*R + repmat(t,25,1);
figure()
scatter3(temp_da(:,1),temp_da(:,2),temp_da(:,3),'red','filled');
hold on
scatter3(tran_data(:,1),tran_data(:,2),tran_data(:,3),'blue','filled')
set(gca, 'FontName', 'Times New Roman')
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

%%
sum_err = 0;

for i=1:size(tran_data,1)
    temp_err = norm(tran_data(i,:)-temp_da(i,:));
    sum_err = temp_err + sum_err;
end

fre = sum_err/25;

