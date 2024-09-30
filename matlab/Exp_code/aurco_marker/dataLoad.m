close all
clc

%对于类似的txt文件，不含有字符，只有数字

num = 11; 
data_aa = cell(11,1);
data_bb = cell(11,1);
for i = 1:11
    fileName0 = ['C:\Users\Lenovo\Desktop\MJ_re-exp\Marker2\tip_cali\0\',int2str(i),'.txt'];
    fileName1 = ['C:\Users\Lenovo\Desktop\MJ_re-exp\Marker2\tip_cali\1\',int2str(i),'.txt'];
    
    data_aa{i} = load(fileName0);
    data_bb{i} = load(fileName1);
end

% fileName0 = 'E:\C++Projects\CenterLab\CenterLab\0-1.txt';
% fileName1 = 'E:\C++Projects\CenterLab\CenterLab\1-1.txt';
% 
% [col1,col2,col3] = textread(fileName0,'%*c %f %*c %f %*c %f %*c',-1);
% plane0 = [col1 col2 col3];
% 
% [col4,col5,col6] = textread(fileName1,'%*c %f %*c %f %*c %f %*c',-1);
% plane1 = [col4 col5 col6];



