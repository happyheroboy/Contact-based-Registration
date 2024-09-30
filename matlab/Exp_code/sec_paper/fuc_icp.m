function [R_final,t_final] = fuc_icp(realpoints,virtualpoint)
%% 精配准
% 初始化参数
max_iterations = 20;
tolerance = 1e-6;

R_final = eye(3);
t_final = [0 0 0]';
% num = size(realpoints);
for iteration = 1:max_iterations
    fprintf('ICP 迭代 %d / %d\n', iteration, max_iterations);
    trans_transformed = (R_final * realpoints' + t_final)';

    indices = knnsearch(virtualpoint, trans_transformed);
    nearest_points = virtualpoint(indices, :);

    temp_d2 = (trans_transformed - nearest_points).^2;
    d2 = temp_d2(:,1)+temp_d2(:,2)+temp_d2(:,3);
    sortedNumbersWithIndex = sortrows([(1:length(d2))' d2],2);
    index = sortedNumbersWithIndex(1:40,1);

    rmse = sqrt(mean(sum((trans_transformed(index,:) - nearest_points(index,:)).^2, 2)));
    fprintf('当前 RMSE: %f\n', rmse);

    trans_right_centered = trans_transformed(index,:)  - mean(trans_transformed(index,:));
    nearest_points_centered = nearest_points(index,:) - mean(nearest_points(index,:));
    H = trans_right_centered' * nearest_points_centered;
    [U, ~, V] = svd(H);
    R = V * U';
    t = mean(nearest_points)' - R * mean(trans_transformed)';

    R_final = R * R_final;
    t_final = t_final + t;

    if rmse < tolerance
        fprintf('ICP 收敛，迭代终止。\n');
        break;
    end
end

end

