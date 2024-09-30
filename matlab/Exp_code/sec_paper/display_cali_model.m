clc
close all

file = "C:\Users\Lenovo\Desktop\probe_1102.model";
% flie = "C:\Users\Lenovo\Desktop\XXZ\【Contact Registration】\实验数据\2024_1_29\\instrument_cali\p1.model";
fid = importdata(flie);

p123 = fid(1,:);
index = size(fid(1,:),2)/4;
point3D_XYZ = [];
for i = 1:index
    temp_P = p123(1,((i-1)*4+1):4*i);
    point3D_XYZ = [point3D_XYZ; temp_P];
end

removed_id =[168,144,96,72,48,24,177,153,129,105,57];
tps =[];
for i =1:size(point3D_XYZ,1)
    t_p =point3D_XYZ(i,:);
    aa = find(t_p(:,1)==removed_id);
    if isempty(aa)
       tps =[tps;t_p(:,2:4)];
    end
end

point3D_XYZ = tps;

figure()
s = scatter3(point3D_XYZ(:,1),point3D_XYZ(:,2),point3D_XYZ(:,3),"red","filled");

s.SizeData = 15;
axis equal
grid off


aa = point3D{1};

figure()
s = scatter3(aa(:,1),aa(:,2),aa(:,3),"blue","filled");
s.SizeData = 30;
axis equal
grid off
