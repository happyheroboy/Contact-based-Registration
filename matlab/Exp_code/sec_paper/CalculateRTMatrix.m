function [R,t] = CalculateRTMatrix(P,X)
    %计算点集均值
    centroidp = mean(P);
    centroidx = mean(X);

    %去中心化/去质心化
    p1=bsxfun(@minus,P,centroidp);
    x1=bsxfun(@minus,X,centroidx);

    %计算点集协方差
    H=p1'*x1;

    [u ,~ ,v] = svd(H);
    R=u*v';
    if det(R)<0
        v = [1,1,-1;1,1,-1;1,1,-1].*v;
%         v=[v(:,1:2) v(:,3)*-1];
        R=u*v';
    end    

    %计算平移向量
    t=centroidx-centroidp*R;
end

