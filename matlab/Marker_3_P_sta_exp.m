reset_toolbox;
clc
close all
load('calibrationSession_9_18.mat')
load('planar_aruco_marker_model.mat')
stereoParams = calibrationSession.CameraParameters; 

Model = point3D_XYZ;
tip_points = [];
total_err = [];

for i = 1:1000
    fileName0 = ['.\Data\Marker-3-P\0\',int2str(i),'.txt'];
    fileName1 = ['.\Data\Marker-3-P\1\',int2str(i),'.txt'];
    
    ptListl = load(fileName0);
    ptListr = load(fileName1);

    if isempty(ptListl)||isempty(ptListr)
        continue;
    end

    ptListl = [ ptListl(:,2) ptListl(:,1) ptListl(:,3)];
    ptListr = [ ptListr(:,2) ptListr(:,1) ptListr(:,3)];

    [c,ia,ib]=intersect(ptListl(:,3),ptListr(:,3));
    if isempty(c)
        continue;
    end

    [point3D,err]=triangulate(ptListl(ia,1:2),ptListr(ib,1:2),stereoParams);
    point3D=[point3D ptListl(ia,3)];
    total_err = [total_err;err];

    [c,ia1,ib1] = intersect(Model(:,4),point3D(:,4));

    [r_1,t_1]  = CalculateRTMatrix(Model(ia1,1:3),point3D(ib1,1:3));
    trans_model = Model(:,1:3)*r_1 + repmat(t_1,5,1);
    tip_points = [tip_points; trans_model(5,:)];

end

std_x = std(tip_points(:,1));
std_y = std(tip_points(:,2));
std_z = std(tip_points(:,3));
std_values = [std_x std_y std_z];
