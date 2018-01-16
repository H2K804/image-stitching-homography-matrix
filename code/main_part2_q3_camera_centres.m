% part 2 , question 3 Camera centres

C1 = load('library1_camera.txt');
C2 = load('library2_camera.txt');

matches = load('library_matches.txt');

%Camera centres for 1st position
[U,D,V] = svd(C1,0);
cam1 = V(:,end);

%Camera centres for 2nd position
[U,D,V] = svd(C2,0);
cam2 = V(:,end);

%dividing x,y,z by w to get camera coordinates
cam1 = cam1./cam1(4);
cam2 = cam2./cam2(4);

% cam1xyz = cam1(1:3);
% disp(cam1xyz);
% 
% cam2xyz = cam2(1:3);
% disp(cam2xyz);

N = size(matches,1);

%matches in 3D plane
points3d = zeros(4,N);
x1 = matches(:,1:2);
x2 = matches(:,3:4);
for i=1:N
    A = [x1(i,1).'*C1(3,:) - C1(1,:);
         x1(i,2).'*C1(3,:) - C1(2,:);
         x2(i,1).'*C2(3,:) - C2(1,:);
         x2(i,2).'*C2(3,:) - C2(2,:)];
    [U,D,V] = svd(A,0);
    points3d(:,i) = V(:,end);
    points3d(:,i) = points3d(:,i)./points3d(4,i);
end

%disp(points3d);


%Plotting
figure, plot3(points3d(1,:),points3d(2,:),points3d(3,:),'.');
hold on
xlabel('z');
ylabel('x');
zlabel('y');
scatter3(cam1(1),cam1(2),cam1(3),'r','fill');
scatter3(cam2(1),cam2(2),cam2(3),'g','fill');
legend('Data','Camera 1','Camera 2')
axis equal

%residuals

x1T = [matches(:,1:2)';ones(1,N)];
x2T = [matches(:,3:4)';ones(1,N)];
projx = (C1*points3d);
projx(1,:) = projx(1,:)./projx(3,:);
projx(2,:) = projx(2,:)./projx(3,:);
projx(3,:) = projx(3,:)./projx(3,:);
r1 = sum(sqrt(sum((x1T - projx).^2)));
disp(r1);
projx2 = (C2*points3d);
projx2(1,:) = projx2(1,:)./projx2(3,:);
projx2(2,:) = projx2(2,:)./projx2(3,:);
projx2(3,:) = projx2(3,:)./projx2(3,:);
r2 = sum(sqrt(sum((x2T - projx2).^2)));
disp(r2);
r=r1+r2;
disp(r);



