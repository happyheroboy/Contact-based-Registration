function  [minDistance,nearestPoint] = findNearestPts(point,pointList)
% ����㵽���е�ľ���
distances = sqrt(sum((pointList - point).^2, 2));

% �ҵ�����ĵ�������;���
[minDistance, nearestPointIndex] = min(distances);
nearestPoint = pointList(nearestPointIndex, :);
end