function [t_err,shortestDistances] = overlapDistance(new_traRcloud,new_points,faces)

shortestDistances = zeros(size(new_traRcloud,1), 1);
for i=1:size(new_traRcloud,1)

    point = new_traRcloud(i,:);

    % 获取所有三角面片的顶点坐标
    vertices1 = new_points(faces(:, 1), :);
    vertices2 = new_points(faces(:, 2), :);
    vertices3 = new_points(faces(:, 3), :);

    % 计算所有三角面片的法向量
    normals = cross(vertices2 - vertices1, vertices3 - vertices1);
    normals = normals ./ vecnorm(normals, 2, 2);

    % 计算点到所有三角面片的投影点
    projectedPoints = point - dot(point - vertices1, normals, 2) .* normals;

    % 计算投影点在三角形内部的条件
    barycentric = cartesianToBarycentric(projectedPoints, vertices1, vertices2, vertices3);
    insideTriangle = all(barycentric >= 0, 2) & sum(barycentric, 2) <= 1;

    % 计算投影点在三角形内部的最近距离
    insideDistances = vecnorm(projectedPoints(insideTriangle, :) - point, 2, 2);

    % 计算点与所有顶点的距离
    vertexDistances = vecnorm(new_points - point, 2, 2);

    % 计算最终的最近距离
    minDistance = min([insideDistances; vertexDistances]);

    shortestDistances(i) = minDistance;

end

t_err = sum(shortestDistances(:));

end

% 从笛卡尔坐标转换为重心坐标
function barycentric = cartesianToBarycentric(points, vertices1, vertices2, vertices3)
    v0 = vertices2 - vertices1;
    v1 = vertices3 - vertices1;
    v2 = points - vertices1;
    
    d00 = dot(v0, v0, 2);
    d01 = dot(v0, v1, 2);
    d11 = dot(v1, v1, 2);
    d20 = dot(v2, v0, 2);
    d21 = dot(v2, v1, 2);
    
    denom = d00 .* d11 - d01 .* d01;
    
    barycentric = [ ...
        (d11 .* d20 - d01 .* d21) ./ denom, ...
        (d00 .* d21 - d01 .* d20) ./ denom, ...
        1 - (d11 .* d20 - d01 .* d21 + d00 .* d21 - d01 .* d20) ./ denom ...
    ];
end
