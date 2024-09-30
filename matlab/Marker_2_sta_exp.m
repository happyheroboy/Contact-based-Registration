
reset_toolbox;
close all;
load('calibrationSession_8-26.mat')
load('probe_model.mat')
stereoParams = calibrationSession.CameraParameters;

%% 读取包含标记的视频
% read a video containing marker
vidObj_0 = VideoReader('.\Data\Marker-2\Recording0.avi');
vidObj_1 = VideoReader('.\Data\Marker-2\Recording1.avi');

A = [];
framesNum_0 = vidObj_0.NumFrames;
framesNum_1 = vidObj_1.NumFrames;

Model = point3D_XYZ;
tip_pts = [];
for i=1:framesNum_0

    fprintf('第%d帧\n',i);

    frame0=read(vidObj_0,i);
    frame1=read(vidObj_1,i);
    
    img0 = im2double(rgb2gray(frame0));
    img1 = im2double(rgb2gray(frame1));
    
    img0 = undistortImage(img0,stereoParams.CameraParameters1);
    img1 = undistortImage(img1,stereoParams.CameraParameters2);

    %% 识别中的特征点
    [ptList0,~] = chess_detector(img0);
    [ptList1,~] = chess_detector(img1);
    
    ptList0 = [ptList0(:,2),ptList0(:,1)];
    ptList1 = [ptList1(:,2),ptList1(:,1)];
    
    ptList0 = sortrows(ptList0,2);
    ptList1 = sortrows(ptList1,2);
    if (size(ptList0,1)~=3)||(size(ptList1,1)~=3)
        continue
    end
   
    %% 三角测量
    [point3D,err] = triangulate(ptList0,ptList1,stereoParams);
    A = [A;mean(err)];

    %% 跟踪
    [r_1,t_1]  = CalculateRTMatrix(Model(1:3,:),point3D);
    trans_model = Model(:,1:3)*r_1 + repmat(t_1,4,1);
    tip_pts = [tip_pts;trans_model(4,:)];
end

std_x = std(tip_pts(:,1));
std_y = std(tip_pts(:,2));
std_z = std(tip_pts(:,3));

std_values = [std_x std_y std_z];
