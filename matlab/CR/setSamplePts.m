function [samplePoints] = setSamplePts(centerPoint,step,n)
%设置采样点
%centerPoint中心点
%step：步长（一个格的边长）
%n：网格规格 n*n

range = (n/2)*step;  %(2023.7.21修改)
[X, Y] = meshgrid(centerPoint(1)-range:step:centerPoint(1)+range, centerPoint(2)-range:step:centerPoint(2)+range);
Z = repmat(centerPoint(3),size(X,1),size(X,1));
samplePoints = [X(:),Y(:),Z(:)];

end