close all;
%Ileft = imread('../data/part1/uttower/left.jpg');
%Iright = imread('../data/part1/uttower/right.jpg');
Ileft = imread('../data/part1/hill/1.jpg');
Iright = imread('../data/part1/hill/2.jpg');
%converting image to grayscale
IleftGray = rgb2gray(Ileft);
IrightGray = rgb2gray(Iright);

%converting image to double values using im2double
Ileftdoub = im2double(IleftGray);
Irightdoub = im2double(IrightGray);

%zero padding
imL = padarray(Ileftdoub,[10 10],0,'both');
imR = padarray(Irightdoub,[10 10],0,'both');

%detecting corners using Harris detector
[cimL, rL, cL] = harris(imL, 3, 0.05, 3, 1);
[cimR, rR, cR] = harris(imR, 3, 0.05, 3, 1);

%forming descriptors for each detected point and flattening
descL = zeros(size(rL,1), (2*7+1)^2);
descR = zeros(size(rR,1), (2*7+1)^2);

for i = 1:size(rL,1)
        descL(i,:) = reshape(imL(rL(i)-7:rL(i)+7, cL(i)-7:cL(i)+7), 1, 225);
end

for i = 1:size(rL,1)
        descR(i,:) = reshape(imR(rR(i)-7:rR(i)+7, cR(i)-7:cR(i)+7), 1, 225);
end

%calcualting distance between descriptors
distMat = zeros(size(descL,1),size(descR,1));
distMat = dist2(descL , descR);

threshold = 5;
matches = [];

for i = 1:size(distMat,1)
    for j = 1:size(distMat,2)
        if(distMat(i,j) < threshold)
            matches = [matches; i, rL(i), cL(i), j, rR(j), cR(j)];
            %matches = [matches; rL(i), cL(i), rR(j), cR(j)];
        end
    end
end

% RANSAC method
iterations = 200;
thres = 0.5;
    
matchSize = size(matches, 1);

n = 1;

for j = 1:iterations-1
    %select random points from matches
    inliers = randsample(matchSize, 4);
    %compute A matrix
    A = [];
    for i = 1:4
        matchx = matches(inliers(i), :);
        xT = [matchx(3), matchx(2), 1];
        A = [A; xT*0, xT, xT*(-matchx(5))];
        A = [A; xT, xT*0, xT*(-matchx(6))];
    end
    
    %find homography
    [U, D, V] = svd(A,0);
    H = reshape(V(:,end), 3, 3)';
    
    num_inliers = 0;
    inliers = [];
    residual = [];
    
    for i = 1:matchSize
        X =  H * [matches(i, 3); matches(i, 2); 1];
        xN = X(1)/X(3);
        yN = X(2)/X(3);
        if(dist2([xN,yN], [matches(i, 6),matches(i, 5)]) < thres)
            num_inliers = num_inliers+1;
            inliers = [inliers; i];
            %calculate distnace between actual and predicted
            resDist = dist2([xN,yN], [matches(i, 6),matches(i, 5)]);
            residual = [residual; resDist];
        end
    end
end

mean_residual = mean(residual);
disp("***********^^^^^^^^^^^^^^^^^^**************");
disp(mean_residual);

final_matches = [];
for i = 1:num_inliers
    final_matches = [final_matches; matches(inliers(i), :)];
end

disp(num_inliers);


% ----------------------------------joining the images---------------------------------


figure();
hold on;

tform = maketform('projective', H);
[IleftT, xdataimT, ydataimT]=imtransform(Ileft, tform, 'XYScale',1);
xdataout=[min(1,xdataimT(1)) max(size(Ileft,2),xdataimT(2))];
ydataout=[min(1,ydataimT(1)) max(size(Iright,1),ydataimT(2))];
img_left=imtransform(Ileft,T,'nearest','XData',xdataout,'YData',ydataout,'XYScale',1);
img_right=imtransform(Iright,maketform('affine',eye(3)),'nearest','XData',xdataout,'YData',ydataout);
[new_height, new_width, tmp] = size(img_left);

imshow(img_right);
figure();
imshow(img_left);

%REFERENCE:
%http://dcyoung.weebly.com/fundamental-matrix--triangulation.html
%https://github.com/jyt0532/Image-Stitching

