clc
close all
reset_toolbox
% the second static experiment, evaluating the accuracy under different
% distances and occlusion ratios.

%%
% distance 1600 mm
oc_dd2 = load(".\Data\occ_1600_pinpoints.txt");
row_size = 120;
num_groups = 1200 / row_size;

grouped_data = mat2cell(oc_dd2, repmat(row_size, 1, num_groups), 3);

all_e1 = cell(5,1);
% compute error
for i = 1:5
    tempD1 = grouped_data{2*i-1};
    tempD2 = grouped_data{(2*i)};
    distances = vecnorm(tempD1 - tempD2, 2, 2);
    temp_err = rmoutliers(abs(100-distances));
    all_e1{i} = temp_err(1:100,:);
end

%% distance 1800 mm
oc_dd3 = load(".\Data\occ_1800_pinpoints.txt");
row_size = 120; 
num_groups = 1200 / row_size;  
grouped_data = mat2cell(oc_dd3, repmat(row_size, 1, num_groups), 3);

err_all1 = [];
all_e2 = cell(5,1);
% compute error
for i = 1:5
    tempD1 = grouped_data{2*i-1};
    tempD2 = grouped_data{(2*i)};
    distances = vecnorm(tempD1 - tempD2, 2, 2);
    temp_err = rmoutliers(abs(100-distances));
    all_e2{i} = temp_err(1:100,:);
    err_all1 = [err_all1 temp_err(1:100,:)];
end

%% draw bar figure
x = 1:5;

dataset =[];
RMSE = [];
STD = [];
for ii = 1:5
    temp_d1 = all_e1{ii};
    temp_d2 = all_e2{ii};
    dataset = [dataset; [mean(temp_d1) mean(temp_d2)]];
    RMSE = [RMSE; [rms(temp_d1),rms(temp_d2)]];
    STD = [STD; [std(temp_d1),std(temp_d2)]];
end

%%
GO = bar(x,dataset,1,'EdgeColor','k','LineWidth',1.5);
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

GO(1).FaceColor = C1;
GO(2).FaceColor = C2;
set(hE, 'LineStyle', 'none', 'Color', 'k','LineWidth', 2)

%%
set(gca, 'Box', 'on', ...                                       
        'XGrid', 'off', 'YGrid', 'off', ...                       
        'TickDir', 'in', 'TickLength', [.01 .01], ...            
        'XMinorTick', 'off', 'YMinorTick', 'off', ...             
        'XColor', [.1 .1 .1],  'YColor',[.1 .1 .1],...            
        'YTick', 0:0.1:0.5,...                                     
        'Ylim' , [0 0.3], ...                                    
        'Xticklabel',{'0%' '20%' '40%' '60%' '80%'},...
        'Yticklabel',{[0:0.1:0.3]})                              

set(gca,'LineWidth', 2) 
hLegend = legend([GO(1),GO(2)], ...
                  '1600 mm', '1800 mm', ...
                 'Location', 'northwest');
P = hLegend.Position;
hLegend.Position = P;
set(gca, 'FontName', 'Times New Roman')
xlabel(gca,'Occlusion ratio')
ylabel(gca,'Error (mm)')
set(gca, 'FontSize', 16)

%-----import image
exportgraphics(gca,'sta_exp_res.png');
