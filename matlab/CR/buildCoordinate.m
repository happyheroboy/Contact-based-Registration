function [R_XYZ] = buildCoordinate(angle_gammar,Z_axis)

% Z_axis = [x,y,z]; % ��Z�᷽��
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