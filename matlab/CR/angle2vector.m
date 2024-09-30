function [R_XYZ] = angle2vector(angle_alpha,angle_gammar,isLeft)
%�ú����ǽ�����ת��Ϊһ����������������Z_axis�����µ�������C�� 
% angle_alpha ǰ���  angle_gammar ��չ��
% Z_axis ��������
% isLeft �Ƿ�������
% R_XYZ ԭ������ϵC��������ϵC'֮���ת����ϵ

if isLeft
    x = sin(pi/2-angle_alpha)*sin(angle_gammar);
    y = -cos(pi/2-angle_alpha);
    z = -sin(pi/2-angle_alpha)*cos(angle_gammar);
else
    x = -sin(pi/2-angle_alpha)*sin(angle_gammar);
    y = -cos(pi/2-angle_alpha);
    z = -sin(pi/2-angle_alpha)*cos(angle_gammar);
end

Z_axis = [x,y,z]; % ��Z�᷽��
if angle_gammar==pi/2
    temmp_vec = [0,1,0];
else
    temmp_vec = [1,0,0];
end

Y_axis = cross(Z_axis,temmp_vec);% ��Y�᷽��
X_axis = cross(Z_axis,Y_axis);% ��X�᷽��

%��λ��
Z_axis = Z_axis/norm(Z_axis);
Y_axis = Y_axis/norm(Y_axis);
X_axis = X_axis/norm(X_axis);

R_XYZ = [X_axis', Y_axis', Z_axis'];
end

function [R_XYZ] = angle2vector2(angle_alpha,angle_gammar,isLeft)
%�ú����ǽ�����ת��Ϊһ����������������Z_axis�����µ�������C�� 
% angle_alpha ǰ���  angle_gammar ��չ��
% Z_axis ��������
% isLeft �Ƿ�������
% R_XYZ ԭ������ϵC��������ϵC'֮���ת����ϵ

if isLeft
    x = -sin(angle_gammar);
    y = sin(angle_alpha);
    z = sqrt(1-x^2-y^2);
else
    x = sin(angle_gammar);
    y = sin(angle_alpha);
    z = sqrt(1-x^2-y^2);
end

Z_axis = -[x,y,z]; % ��Z�᷽��
% Z_axis = Z_axis/norm(Z_axis);
Y_axis = [1, -Z_axis(1)/(Z_axis(2)+Z_axis(3)), -Z_axis(1)/(Z_axis(2)+Z_axis(3))];% ��Y�᷽��
X_axis = cross(Z_axis,Y_axis);% ��X�᷽��

%��λ��
Z_axis = Z_axis/norm(Z_axis);
Y_axis = Y_axis/norm(Y_axis);
X_axis = X_axis/norm(X_axis);

R_XYZ = [X_axis', Y_axis', Z_axis'];
end



