reset_toolbox;
clc
close all
load('Marker-3-C model.mat')
load('calibrationSession_9_10.mat')
stereoParams = calibrationSession.CameraParameters; 
%%
Model = point3D_XYZ;
tip_points = [];
total_err = [];

for i = 1:25
    fileName0 = ['.\Data\Marker-3-C\00\',int2str(i),'.txt'];
    fileName1 = ['.\Data\Marker-3-C\11\',int2str(i),'.txt'];
    
    ptListl = load(fileName0);
    ptListr = load(fileName1);

    if isempty(ptListl)||isempty(ptListr)
        continue;
    end

    ptListl = [ ptListl(:,2) ptListl(:,1) ptListl(:,3)];
    ptListr = [ ptListr(:,2) ptListr(:,1) ptListr(:,3)];

    %% 三角测量

    [c,ia,ib]=intersect(ptListl(:,3),ptListr(:,3));
    if isempty(c)
        continue;
    end

    %%计算第一对匹配点的三维值
    [point3D,err]=triangulate(ptListl(ia,1:2),ptListr(ib,1:2),stereoParams);
    point3D=[point3D ptListl(ia,3)];
    total_err = [total_err;mean(err)];

    [c,ia1,ib1] = intersect(Model(:,4),point3D(:,4));

    [r_1,t_1]  = CalculateRTMatrix(Model(ia1,1:3),point3D(ib1,1:3));
    trans_model = Model(:,1:3)*r_1 + repmat(t_1,14,1);
    tip_points = [tip_points; trans_model(14,:)];

end

temp_da = tip_points;
% temp_da = [tip_points(21:25,:);tip_points(1:20,:)];
% 创建X轴和Y轴的坐标向量
x = 0:25:100; % 0, 25, 50, 75, 100
y = 0:25:100; % 0, 25, 50, 75, 100

% 使用meshgrid函数创建网格
[X, Y] = meshgrid(x, y);
Z=repmat(100,5);

sim_data = [X(:) Y(:) Z(:)];

% 显示网格
figure()
scatter3(X(:), Y(:), Z(:), 200,'filled');


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

% grid on;
% axis equal; % 设置坐标轴比例相等
%%
% 计算FRE
sum_err = 0;

for i=1:size(tran_data,1)
    temp_err = norm(tran_data(i,:)-temp_da(i,:));
    sum_err = temp_err + sum_err;
end

fre = sum_err/25;