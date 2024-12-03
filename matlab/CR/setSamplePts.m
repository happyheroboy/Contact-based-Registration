function [samplePoints] = setSamplePts(centerPoint,step,n)
%���ò�����
%centerPoint���ĵ�
%step��������һ����ı߳���
%n�������� n*n

range = (n/2)*step;  %(2023.7.21�޸�)
[X, Y] = meshgrid(centerPoint(1)-range:step:centerPoint(1)+range, centerPoint(2)-range:step:centerPoint(2)+range);
Z = repmat(centerPoint(3),size(X,1),size(X,1));
samplePoints = [X(:),Y(:),Z(:)];

end