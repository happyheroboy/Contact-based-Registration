close all

model_0= [
1262, 1163, 28;
1264, 1268, 29;
1369, 1268, 30;
1367, 1163, 31;];
model_0 = [model_0(:,2) model_0(:,1) model_0(:,3)];

model_1= [
1282, 1175, 28;
1283, 1281, 29;
1388, 1280, 30;
1387, 1174, 31;];
model_1 = [model_1(:,2) model_1(:,1) model_1(:,3)];

tip_0 = [1224 1734];
tip_1 = [1224 1752];

%% 计算第一对匹配点的三维值
[c,ia,ib] = intersect(model_0(:,3),model_1(:,3));
[point3D_,err] = triangulate(model_0(ia,1:2),model_1(ib,1:2),stereoParams);
point3D_model = [point3D_ model_0(ia,3)];

figure
scatter3(point3D_model(:,1),point3D_model(:,2),point3D_model(:,3),'ro');

[tip_point3D,err] = triangulate(tip_0,tip_1,stereoParams);

point3D_XYZ =[point3D_model; tip_point3D 1000];

%% 投影至图片

[points1, points2, camMatrix1, camMatrix2] = ...
parseInputs(model_0(:,1:2), model_1(:,1:2), stereoParams);

[r_1,t_1]  = SVDICPxiugai(point3D_XYZ(1:4,1:3),point3D_);
trans_model = point3D_XYZ(:,1:3)*r_1 + repmat(t_1,5,1);

%% 显示
points2d = projectPoints(trans_model, camMatrix1);
points2d = points2d';


fileName0 = 'C:\Users\Lenovo\Desktop\MJ_re-exp\Marker-4\model\0\5.bmp';
img0 = im2double(imread(fileName0));
imshow(img0);
hold on
scatter(points2d(:,1),points2d(:,2),100,'r','filled','o','LineWidth',1);