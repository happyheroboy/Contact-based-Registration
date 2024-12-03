function [R,t,err1,err2] = computerTransformation(A,B)

%落点
rotation_center_A = A(:,3);
rotation_center_B = B(:,3);

t = rotation_center_B - rotation_center_A;

new_A = A + repmat(t,1,4); 
new_B = B; 

new_A =  new_A - repmat(rotation_center_B,1,4);
new_B =  new_B - repmat(rotation_center_B,1,3);


% figure();
% plot3(new_A(1,:),new_A(2,:),new_A(3,:),'b');
% hold on
% plot3(new_B(1,:),new_B(2,:),new_B(3,:),'r');

% figure();
% scatter3(new_A(1,1:3),new_A(2,1:3),new_A(3,1:3),'MarkerFaceColor','b');
% hold on
% scatter3(new_B(1,:),new_B(2,:),new_B(3,:),'MarkerFaceColor','r');

%%
%A数据中的两个向量
vec_a1 = new_A(:,1) - new_A(:,3);
vec_a1 = vec_a1/norm(vec_a1);
vec_a2 = new_A(:,2) - new_A(:,3);
vec_a2 = vec_a2/norm(vec_a2);
cross_a1a2 = cross(vec_a1,vec_a2);
cross_a1a2 = cross_a1a2/norm(cross_a1a2);

%B数据中的两个向量
vec_a3 = new_B(:,1) - new_B(:,3);
vec_a3 = vec_a3/norm(vec_a3);
vec_a4 = new_B(:,2) - new_B(:,3);
vec_a4 = vec_a4/norm(vec_a4);
cross_a3a4 = cross(vec_a3,vec_a4);
cross_a3a4 = cross_a3a4/norm(cross_a3a4);

%%
A_M = [vec_a1,vec_a2];
B_M = [vec_a3,vec_a4];

[U_1,~,V_1] = svd(B_M*A_M'); %执行矩阵 A 的奇异值分解，因此 A = U*S*V'

%%
R = U_1*V_1';
val = det(R);

if det(R) < 0
    fprintf("det(R) < R, reflection detected!, correcting for it ...\n");
    V_1(:,3) = V_1(:,3) * -1;
    R = U_1*V_1';
end

trans_newA = R*new_A;
err1 = norm(new_B(:,1) - trans_newA(:,1));
err2 = norm(new_B(:,2) - trans_newA(:,2));

aaa = R*vec_a2; % 向量A
bbb = vec_a4; % 向量B
dot_product = dot(aaa, bbb); % 计算点积
norm_A = norm(aaa); % 计算向量A的模长
norm_B = norm(bbb); % 计算向量B的模长
angle = acos(dot_product / (norm_A * norm_B)); % 计算夹角，单位为弧度
angle_deg = rad2deg(angle); % 将夹角转换为角度
disp(angle_deg); % 显示夹角的角度值

% hold on
% scatter3(trans_newA(1,1:3),trans_newA(2,1:3),trans_newA(3,1:3),'MarkerFaceColor','g');
end