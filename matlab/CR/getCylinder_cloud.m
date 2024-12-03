
function [pointsInCylinder]=getCylinder_cloud(pointCloud,center,radius)
% 假设pointCloud为已有的点云数据，是一个N*3的矩阵，每一行代表一个点的x, y, z坐标
% center为圆柱的中心点，是一个1*3的向量，代表x, y, z坐标
% radius为圆柱的半径

% 计算所有点到圆柱中心的水平距离
distances = (pointCloud(:, 1) - center(1)).^2 + (pointCloud(:, 2) - center(2)).^2;

% 找到那些距离小于或等于半径的点的索引
indices = find(round(distances,4) <= round(radius^2,4));

% 使用这些索引来获取满足条件的点
pointsInCylinder = pointCloud(indices, :);
end
