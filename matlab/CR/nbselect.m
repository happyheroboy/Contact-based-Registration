function [idx,dist] = nbselect(data,part,varargin)
% 功能：构建K-D树，选择不同邻域类型，返回邻域点索引与距离
% 输入：data     - 原始数据(M*3)    
%       part     - 待检索数据(N*3)
%       varargin - 球状、柱状邻域或KNN+半径或个数
%       'Q'  - 球状邻域
%       'Z'  - 柱状邻域
%       'K'  - KNN 
% 输出：idx  - 邻域点索引 
%       dist - 距离
% example：[sph,dist_sph] =  nbselect(data,part,'Q',r_Q);
if varargin{1} == 'Q'
    r_Q = varargin{2};
    [idx,dist] = rangesearch(data(:,1:3),part(:,1:3),r_Q,'Distance','euclidean','NSMethod','kdtree');      
elseif varargin{1} == 'Z'
    r_Z = varargin{2};
    [idx,dist] = rangesearch(data(:,1:2),part(:,1:2),r_Z,'Distance','euclidean','NSMethod','kdtree');  
elseif varargin{1} == 'K'
    k = varargin{2};
    [idx,dist] = knnsearch(data(:,1:3),part(:,1:3),'Distance','euclidean','NSMethod','kdtree','K',k);  
end
end