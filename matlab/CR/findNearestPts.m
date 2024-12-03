function  [minDistance,nearestPoint] = findNearestPts(point,pointList)
% 计算点到所有点的距离
distances = sqrt(sum((pointList - point).^2, 2));

% 找到最近的点的索引和距离
[minDistance, nearestPointIndex] = min(distances);
nearestPoint = pointList(nearestPointIndex, :);
end