
function [pointsInCylinder]=getCylinder_cloud(pointCloud,center,radius)
% ����pointCloudΪ���еĵ������ݣ���һ��N*3�ľ���ÿһ�д���һ�����x, y, z����
% centerΪԲ�������ĵ㣬��һ��1*3������������x, y, z����
% radiusΪԲ���İ뾶

% �������е㵽Բ�����ĵ�ˮƽ����
distances = (pointCloud(:, 1) - center(1)).^2 + (pointCloud(:, 2) - center(2)).^2;

% �ҵ���Щ����С�ڻ���ڰ뾶�ĵ������
indices = find(round(distances,4) <= round(radius^2,4));

% ʹ����Щ��������ȡ���������ĵ�
pointsInCylinder = pointCloud(indices, :);
end
