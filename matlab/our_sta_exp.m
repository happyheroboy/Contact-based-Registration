clc
close all;
reset_toolbox

% 导入数据
data_1 = load(".\Data\stablity.txt");
% 去除噪声
data_12 = rmoutliers(data_1);

std_x = std(data_12(1:1000,1));
std_y = std(data_12(1:1000,2));
std_z = std(data_12(1:1000,3));

bar_x = data_12(1:1000,1);
bar_y = data_12(1:1000,2);
bar_z = data_12(1:1000,3);

mean_x = mean(bar_x);
mean_y = mean(bar_y);
mean_z = mean(bar_z);

bar_xx = bar_x -repmat(mean_x,size(bar_x,1),1);
bar_yy = bar_y -repmat(mean_y,size(bar_x,1),1);
bar_zz = bar_z -repmat(mean_z,size(bar_x,1),1);
figure
h0 = boxplot([bar_xx,bar_yy,bar_zz],'Notch','off','Labels',{'X','Y','Z'},'Widths',0.3,'Whisker',10);
set(h0,'LineWidth',2.5)
ylabel('Error (mm)');
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 15)
set(gca, 'Box', 'on', ...                                         % 边框
         'LineWidth', 2,...                                       % 线宽
         'XGrid', 'off', 'YGrid', 'off', ...                      % 网格
         'TickDir', 'in', 'TickLength', [.015 .015], ...          % 刻度
         'XMinorTick', 'off', 'YMinorTick', 'off', ...            % 小刻度
         'XColor', [.1 .1 .1],  'YColor', [.1 .1 .1])             % 坐标轴颜色

