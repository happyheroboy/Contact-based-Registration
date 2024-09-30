clc
close all
reset_toolbox

dynamic_data = load(".\Data\our_dynamic_exp.txt");
temp_da = [];
for i=1:25
    temp = dynamic_data((60*(i-1)+1):60*i,:);
    temp_da = [temp_da;mean(temp)];
end

%% construct ground truth using grid
x = 0:25:100; % 0, 25, 50, 75, 100
y = 0:25:100; % 0, 25, 50, 75, 100
[X, Y] = meshgrid(x, y);
Z=repmat(100,5);
sim_data = [X(:) Y(:) Z(:)];

%%
[R,t] = CalculateRTMatrix(sim_data,temp_da(1:25,:));
tran_data = sim_data*R + repmat(t,25,1);
figure()
scatter3(temp_da(:,1),temp_da(:,2),temp_da(:,3),120,'red','filled');
hold on
scatter3(tran_data(:,1),tran_data(:,2),tran_data(:,3),120,'blue','filled')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 20)
set(gca, 'Box', 'off', ...                                      
         'LineWidth', 1.5)

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

% grid on;
% axis equal; 
%%
% 计算FRE
sum_err = 0;
for i=1:size(tran_data,1)
    temp_err = norm(tran_data(i,:)-temp_da(i,:));
    sum_err = temp_err + sum_err;
end
fre = sum_err/25;
