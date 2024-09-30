
data_00 = [975	644	60	
1049	646	61	
1047	576	62	
975	573	63	
891	641	56	
965	644	57	
964	572	58	
890	571	59	
1059	647	64	
1132	649	65	
1131	580	66	
1058	577	67	];

data_11 = [1087	727	64	
1161	728	65	
1159	655	66	
1086	655	67	
1002	727	60	
1079	728	61	
1075	655	62	
1002	654	63	
918	727	56	
992	727	57	
991	654	58	
918	652	59	
];
% data_00 =[1062, 478, 18446744073709551612;
% 934, 482, 18446744073709551613;
% 940, 612, 18446744073709551614;
% 1068, 608, 18446744073709551615;];
% 
% data_11 = [1091, 528, 18446744073709551612;
% 960, 535, 18446744073709551613;
% 966, 668, 18446744073709551614;
% 1096, 661, 18446744073709551615;];
% 
% da_id = [12;13;14;15];
% data_00 = [data_00(:,2) data_00(:,1) da_id];
% data_11 = [data_11(:,2) data_11(:,1) da_id];

ptListl = data_00;
ptListr = data_11;

%% 计算第一对匹配点的三维值
[c,ia,ib] = intersect(ptListl(:,3),ptListr(:,3));
[point3D_,err] = triangulate(ptListl(ia,1:2),ptListr(ib,1:2),stereoParams);
point3D_model = [point3D_ ptListl(ia,3)];

figure
scatter3(point3D_model(:,1),point3D_model(:,2),point3D_model(:,3),'ro');

tipPt0 = [583,1664];
tipPt1 = [616,1702];
[tip_point3D,err] = triangulate(tipPt0,tipPt1,stereoParams);

point3D_XYZ =[point3D_model; tip_point3D 1000];

%% 投影至图片

[points1, points2, camMatrix1, camMatrix2] = ...
parseInputs(ptList0, ptList1, stereoParams);

[r_1,t_1]  = SVDICPxiugai(point3D_XYZ(1:4,1:3),point3D_);
trans_model = point3D_XYZ(:,1:3)*r_1 + repmat(t_1,5,1);

%% 显示
points2d = projectPoints(trans_model, camMatrix1);
points2d = points2d';


fileName0 = 'C:\Users\Lenovo\Desktop\MJ_re-exp\08-26 model\0\0.bmp';
img0 = im2double(imread(fileName0));
imshow(img0);
hold on
scatter(points2d(:,1),points2d(:,2),100,'r','filled','o','LineWidth',1);





