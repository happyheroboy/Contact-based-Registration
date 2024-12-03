function [idx,dist] = nbselect(data,part,varargin)
% ���ܣ�����K-D����ѡ��ͬ�������ͣ�������������������
% ���룺data     - ԭʼ����(M*3)    
%       part     - ����������(N*3)
%       varargin - ��״����״�����KNN+�뾶�����
%       'Q'  - ��״����
%       'Z'  - ��״����
%       'K'  - KNN 
% �����idx  - ��������� 
%       dist - ����
% example��[sph,dist_sph] =  nbselect(data,part,'Q',r_Q);
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