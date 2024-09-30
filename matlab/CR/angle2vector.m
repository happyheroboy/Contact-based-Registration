function [R_XYZ] = angle2vector(angle_alpha,angle_gammar,isLeft)
%该函数是将两角转化为一个入射向量，并以Z_axis建立新的坐标轴C’ 
% angle_alpha 前倾角  angle_gammar 外展角
% Z_axis 入射向量
% isLeft 是否是左髋
% R_XYZ 原先坐标系C和新坐标系C'之间的转换关系

if isLeft
    x = sin(pi/2-angle_alpha)*sin(angle_gammar);
    y = -cos(pi/2-angle_alpha);
    z = -sin(pi/2-angle_alpha)*cos(angle_gammar);
else
    x = -sin(pi/2-angle_alpha)*sin(angle_gammar);
    y = -cos(pi/2-angle_alpha);
    z = -sin(pi/2-angle_alpha)*cos(angle_gammar);
end

Z_axis = [x,y,z]; % 新Z轴方向
if angle_gammar==pi/2
    temmp_vec = [0,1,0];
else
    temmp_vec = [1,0,0];
end

Y_axis = cross(Z_axis,temmp_vec);% 新Y轴方向
X_axis = cross(Z_axis,Y_axis);% 新X轴方向

%单位化
Z_axis = Z_axis/norm(Z_axis);
Y_axis = Y_axis/norm(Y_axis);
X_axis = X_axis/norm(X_axis);

R_XYZ = [X_axis', Y_axis', Z_axis'];
end

function [R_XYZ] = angle2vector2(angle_alpha,angle_gammar,isLeft)
%该函数是将两角转化为一个入射向量，并以Z_axis建立新的坐标轴C’ 
% angle_alpha 前倾角  angle_gammar 外展角
% Z_axis 入射向量
% isLeft 是否是左髋
% R_XYZ 原先坐标系C和新坐标系C'之间的转换关系

if isLeft
    x = -sin(angle_gammar);
    y = sin(angle_alpha);
    z = sqrt(1-x^2-y^2);
else
    x = sin(angle_gammar);
    y = sin(angle_alpha);
    z = sqrt(1-x^2-y^2);
end

Z_axis = -[x,y,z]; % 新Z轴方向
% Z_axis = Z_axis/norm(Z_axis);
Y_axis = [1, -Z_axis(1)/(Z_axis(2)+Z_axis(3)), -Z_axis(1)/(Z_axis(2)+Z_axis(3))];% 新Y轴方向
X_axis = cross(Z_axis,Y_axis);% 新X轴方向

%单位化
Z_axis = Z_axis/norm(Z_axis);
Y_axis = Y_axis/norm(Y_axis);
X_axis = X_axis/norm(X_axis);

R_XYZ = [X_axis', Y_axis', Z_axis'];
end



