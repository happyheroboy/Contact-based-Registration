
reset_toolbox;
close all;
load 10x10_for_3x3_6x2_10x1.mat
load('calibrationSession_8-26.mat')
load('square_maker_model.mat')

stereoParams = calibrationSession.CameraParameters; 

%% 读取包含标记的视频
% read a video containing marker
vidObj_0 = VideoReader('.\Data\Marker-1\Recording0.avi');
vidObj_1 = VideoReader('.\Data\Marker-1\Recording1.avi');

framesNum_0 = vidObj_0.NumFrames;
framesNum_1 = vidObj_1.NumFrames;

Model = point3D_XYZ;

tip_points = [];
total_err = [];
for i=1:1050

    fprintf('第%d帧\n',i);

    frame0=read(vidObj_0,i);
    frame1=read(vidObj_1,i);
    
    img0 = im2double(rgb2gray(frame0));
    img1 = im2double(rgb2gray(frame1));

    img0 = undistortImage(img0,stereoParams.CameraParameters1);
    img1 = undistortImage(img1,stereoParams.CameraParameters2);

    expectN = 2*(size(sta,1)+1)*(size(sta,2)+1);
    [ptListl,edgel] = read_marker(img0,sta,5,expectN,3);    
    [ptListr,edger] = read_marker(img1,sta,5,expectN,3);

    if (isempty(ptListl))||(isempty(ptListr))
        continue
    end

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
    %[r_mat,t_mat]  = CalculateRTMatrix(pre3Did(ia,1:3),cur3Did(ib,1:3));

    [r_1,t_1]  = CalculateRTMatrix(Model(ia1,1:3),point3D(ib1,1:3));
    trans_model = Model(:,1:3)*r_1 + repmat(t_1,123,1);
    tip_points = [tip_points; trans_model(123,:)];

end

std_x = std(tip_points(1:1000,1));
std_y = std(tip_points(1:1000,2));
std_z = std(tip_points(1:1000,3));
std_values = [std_x std_y std_z];
