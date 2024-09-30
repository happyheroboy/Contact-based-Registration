clc
close all
% 读取数据
% 自变量
x = 1:5;
% 因变量
% dataset为5*3的矩阵，一行3个为一组，共5组
dataset = [mean(err_11),mean(err_21),mean(err_31);
          mean(err_12),mean(err_22),mean(err_32);
          mean(err_13),mean(err_23),mean(err_33);
          mean(err_14),mean(err_24),mean(err_34);
          mean(err_15),mean(err_25),mean(err_35)];
% 误差矩阵
RMSE = [rms(err_11),rms(err_21),rms(err_31);
rms(err_12),rms(err_22),rms(err_32);    
rms(err_13),rms(err_23),rms(err_33);
rms(err_14),rms(err_24),rms(err_34);
rms(err_15),rms(err_25),rms(err_35);
]; % 下方长度
STD = [std(err_11),std(err_21),std(err_31);
std(err_12),std(err_22),std(err_32);    
std(err_13),std(err_23),std(err_33);
std(err_14),std(err_24),std(err_34);
std(err_15),std(err_25),std(err_35);
];  % 上方长度

%%
% 绘制初始柱状图
GO = bar(x,dataset,1,'EdgeColor','k');
% 添加误差棒
[M,N] = size(dataset);
xpos = zeros(M,N);
for i = 1:N
    xpos(:,i) = GO(1,i).XEndPoints'; % v2019b
end
hold on;
hE = errorbar(xpos, dataset, STD, STD);

%%
C1 = addcolor(92);
C2 = addcolor(23);
C3 = addcolor(42);

%%
% 柱状图赋色
GO(1).FaceColor = C1;
GO(2).FaceColor = C2;
GO(3).FaceColor = C3;
% 误差棒属性
set(hE, 'LineStyle', 'none', 'Color', 'k','LineWidth', 1.2)

%%

% 坐标区调整
set(gca, 'Box', 'off', ...                                        % 边框
        'XGrid', 'off', 'YGrid', 'on', ...                        % 网格
        'TickDir', 'out', 'TickLength', [.01 .01], ...            % 刻度
        'XMinorTick', 'off', 'YMinorTick', 'off', ...             % 小刻度
        'XColor', [.1 .1 .1],  'YColor',[.1 .1 .1],...            % 坐标轴颜色
        'YTick', 0:0.1:1,...                                      % 刻度位置、间隔
        'Ylim' , [0 0.31], ...                                     % 坐标轴范围
        'Xticklabel',{'0%' '20%' '40%' '60%' '80%'},...% X坐标轴刻度标签
        'Yticklabel',{0:0.1:1})                                 % Y坐标轴刻度标签
% Legend 设置   
hLegend = legend([GO(1),GO(2),GO(3)], ...
                 '1400 mm', '1600 mm', '1800 mm', ...
                 'Location', 'northwest');
% Legend位置微调
P = hLegend.Position;
hLegend.Position = P + [0.015 0.01 0 0];
% 字体和字号
set(gca, 'FontName', 'Times New Roman')
xlabel(gca,'Occlusion ratio')
ylabel(gca,'Error (mm)')
set([hXLabel, hYLabel], 'FontName','AvantGarde')
set(gca, 'FontSize', 10)
set([hXLabel, hYLabel], 'FontSize', 11)
set(hTitle, 'FontSize', 11, 'FontWeight' ,'bold')

% 背景颜色
set(gcf,'Color',[1 1 1])

%% 图片输出
figW = figureWidth;
figH = figureHeight;
set(figureHandle,'PaperUnits',figureUnits);
set(figureHandle,'PaperPosition',[0 0 figWfigH]);
fileout = 'test';
print(figureHandle,[fileout,'.png'],'-r300','-dpng');
