function [F residual] = fit_fundamental_1(matches, normalizeFlag)

F = zeros(3,3);
N = size(matches, 1);

rand_inliers = randsample(N,8);

if (normalizeFlag == 1)
    [matches, TL, TR] = normalize(matches);
end

A = [];
for i = 1:size(rand_inliers,1)
    uL = matches(rand_inliers(i), 1);
    vL = matches(rand_inliers(i), 2);
    uR = matches(rand_inliers(i), 3);
    vR = matches(rand_inliers(i), 4); 
    A = [A; uR*uL uR*vL uR vR*uL vR*vL vR uL vL 1];
end

[U,D,V]=svd(A);
F = reshape(V(:,end), 3, 3)';
%rank-2 constrain
[FU,FD,FV] = svd(F,0);
FD(3,3) = 0;
F = FU * FD * FV';

% to denormailize
if (normalizeFlag == 1)
    F = TL'*F*TR;
end

residual = [];
for i = 1:size(matches,1)
    Xres = F * [matches(i, 1) matches(i, 2) 1]';
    residual = [residual; abs([matches(i, 3),matches(i, 4), 1]*Xres)];
end   
disp("****residual****");
disp(sum(residual));




