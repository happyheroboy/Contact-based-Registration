
function [original_points,original_faces]=importModel(filename)
% 导入stl模型
%filename: 模型文件路径

TR = stlread(filename);
%显示髋臼, 3D visualization
figure();
patch('Faces',TR.ConnectivityList,'Vertices',TR.Points,'FaceColor','w');
view(3);

original_points = TR.Points;
original_faces = TR.ConnectivityList;
%%
%将三角形网格细化，将模型简化为球与凹点云之间的最优落点求解
% tic;
% [newPoints, newFace] = trimeshLinearSubdivision(TR.Points',TR.ConnectivityList');
% toc;
% newPoints = newPoints';
% newFace = newFace';
% figure();
% patch('Faces',newFace,'Vertices',newPoints,'FaceColor','w');
% view(3);

%%
%三角面片信息
% Tri = reshape(newPoints(newFace(:),:),[size(newFace,1),9]);
% triM = [Tri(:,1) Tri(:,4) Tri(:,7) Tri(:,2) Tri(:,5) Tri(:,8) Tri(:,3) Tri(:,6) Tri(:,9)];
% 
% max_edge = 0;
% tic;
% for i = 1:length(triM)
%      triMesh = triM(i,:);
%      point_A = triMesh(:,1:3);
%      point_B = triMesh(:,4:6);
%      point_C = triMesh(:,7:9);
%      edge_1 =abs(norm(point_A - point_B));
%      edge_2 =abs(norm(point_B - point_C));
%      edge_3 =abs(norm(point_A - point_C));
%      max_e = max([edge_1,edge_2,edge_3]);
%      %min_e = min([edge_1,edge_2,edge_3]);
%      if max_e > max_edge
%          max_edge = max_e;
%      end
% end
% toc;
end