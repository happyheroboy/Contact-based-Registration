function [ptList,ledge] = chess_detector(img)
%CHESS_DETECTOR 棋盘格检测
%   此处显示详细说明

if ~exist('r','var')
    r = 6;
end
if ~exist('expectN','var')
    expectN = 100;
end
if ~exist('sigma','var')
    sigma = 3;
end

% ptList = preFilter(img,r,expectN,sigma);
pt=detectHarrisFeatures(img);
ptList=[pt.Location(:,2),pt.Location(:,1)];
% figure;
% imshow(img);
% hold on;
% % 绘制边 draw edges
% % Y = ptList(:,1);
% % X = ptList(:,2);
% % plot(X(edge'),Y(edge'),'LineWidth',3,'Color','g');
% % 绘制点 draw dots
% scatter(ptList(:,2),ptList(:,1),100,'r','filled','o','LineWidth',1);

[ptList,ledge] = ptRefine(img,ptList,r);
% figure;
% imshow(img);
% hold on;
% % 绘制边 draw edges
% % Y = ptList(:,1);
% % X = ptList(:,2);
% % plot(X(edge'),Y(edge'),'LineWidth',3,'Color','g');
% % 绘制点 draw dots
% scatter(ptList(:,2),ptList(:,1),100,'r','filled','o','LineWidth',1);


end



function ptList = preFilter(img,r,expectN,sigma)

    % "G" 表示在向量和中相互抵消的梯度的总模量, 在对比度高且对称的图案(交叉点)附近较高.
    % "G" is the power of the cancelled out gradients, which is large near
    % high contrast and symmetric patterns (cross point).
    [Gx,Gy] = imgradientxy(img);
    Gpow = imgaussfilt((Gx.^2+Gy.^2).^0.5,sigma);
    Gsum = (imgaussfilt(Gx,sigma).^2 + imgaussfilt(Gy,sigma).^2).^0.5;
    G = Gpow-Gsum;
    
    % 非极大值抑制
    % Non-maximum suppression    
    G(imdilate(G,strel('square',3))~=G)=0;

    % 挑选"expectN"个"G"值最高的点
    % Pick the candidates with top-"expectN" "G" value
    G(1:r,:) = 0; G(end-r+1:end,:) = 0;
    G(:,1:r) = 0; G(:,end-r+1:end) = 0;
    G_sort = G(:); G_sort(G_sort<0.1)=[];
    G_sort = sort(G_sort,'descend');
    [im,in] = ind2sub(size(G),find(G>=G_sort(min(expectN,size(G_sort,1)))));
    ptList = [im,in];

end




function [ptList,ledgeList] = ptRefine(img,list,r)
    
    % 基于梯度优化交叉点位置
    % refine cross point locations based on gradient
    [Gx,Gy] = imgradientxy(img);
    for iter = 1 : 2
        illegal_first = (any(list<r+2,2) | list(:,1)>size(img,1)-r-2 | list(:,2)>size(img,2)-r-2);
        for it = 1 : size(list,1)            
            if illegal_first(it)
                continue;
            end
        
            im = round(list(it,1));
            in = round(list(it,2));
%             fprintf('%d\n',it);

            [M,N] = ndgrid(im-r:im+r,in-r:in+r);
            Gm = Gy(im-r:im+r,in-r:in+r);
            Gn = Gx(im-r:im+r,in-r:in+r);

            G = [Gm(:),Gn(:)];
            p = sum([M(:),N(:)].*G,2);
            list(it,:) = (G\p)';
        end
        % 清除靠近边界的点
        % remove the detected points near boundaries
        illegal = (any(list<r+2,2) | list(:,1)>size(img,1)-r-2 | list(:,2)>size(img,2)-r-2);
        list(illegal,:)=[];
    end

    % 基于超平面模型（二元二次方程）计算脊的朝向
    % 基于脊的朝向生成标准模板, 计算相关度以排除假阳性检测
    % calculate the ledge angles based on hyperplane model (binary
    % quadratic equation)
    % build standard template based on the ledge angles, and calculate
    % correlation score to reject false positive
    ledge = zeros(size(list,1),2);
    corr = zeros(size(list,1),1);
    angBias = zeros(size(list,1),1);
    
    [u,v] = meshgrid(-r:r);
    ut = u(:);  vt = v(:);
    A = [ut.^2,ut.*vt,vt.^2,ones(size(ut,1),1)];
    for it = 1 : size(list,1)
        iy = round(list(it,1));
        ix = round(list(it,2));
        [X,Y] = meshgrid(ix-r-1:ix+r+1,iy-r-1:iy+r+1);
        b = interp2(X,Y,img(iy-r-1:iy+r+1,ix-r-1:ix+r+1),...
                    list(it,2)+u,list(it,1)+v);
        c = A\b(:);
        theta = rad2deg(atan(roots([c(3),c(2),c(1)])));        
        
        if ~isreal(theta)
            ledge(it,:) = nan(1,2);
            continue;
        end
        
        ledge(it,:) = theta;
        template = sign(A(:,1:3)*c(1:3));
        corr(it) = corr2(template,b(:));
        angBias(it) = rad2deg(abs(angdiff(deg2rad([theta(1),theta(2)]))));
        angBias(it) = abs(angBias(it)-90);
        
        % 区分跳白和跳黑的ledge, 这取决于"k"的符号.
        % Identify the white/black jump angle.
        % It depends on the signum of "k".
        % "k"为正时, "theta"中较大的值为跳白线, 反之则为跳黑线. 
        % When "k" is positive, the larger one in "theta" is the white jump
        % angle, otherwise, the smaller one is.
        sign_k = sign(theta(1)*theta(2)*c(1));
        if sign_k == 1
            ledge(it,:) = [max(theta),min(theta)];
        else
            ledge(it,:) = [min(theta),max(theta)];
        end
        
    end
    
    % 删除不足2条脊、低相关度和脊夹角过低的点.
    % Remove the detected points with ledges less than 2, low correlation
    % socre or sharp included angle.
    idx = isnan(ledge(:,1)) ...
        | corr < max(corr)-0.12...
        | angBias > 20;
    list(idx,:) = [];
    ledge(idx,:) = [];

    % 拼合相互距离较小的点
    % Merge the points closed to each other
    if size(list,1)<2
        ptList = list;
        ledgeList = ledge;
        return;
    end
    ptTree = linkage(list,'average','chebychev');
    ptIdx = cluster(ptTree,'Cutoff',2,'Criterion','distance');
    
    ptList = zeros(length(unique(ptIdx)),2);
    ledgeList = zeros(length(unique(ptIdx)),2);
    for it = 1 : length(unique(ptIdx))
       ptList(it,:) = mean(list(ptIdx==it,:),1);
       ledgeList(it,:) = mean(ledge(ptIdx==it,:),1);
    end

end
