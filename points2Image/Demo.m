%Qucik Program to demo the use of points2Image

%% generate a set of 3d points
z = peaks;
x = repmat(1:size(z,1),size(z,1),1);
y = x';
c = z - min(z(:));
c = c./max(c(:));
c = round(255*c) + 1;
cmap = colormap(jet(256));
c = cmap(c,:);

points = [x(:),y(:),z(:),c];

%% setup 

%setup camera
cam = 200;

%setup image
imageSize = [1000,1000];

%create a tform matrix
angles = [5,-5,50]*pi/180;
position = [-25,-25,50];
tform = eye(4);
tform(1:3,1:3) = angle2dcm(angles(1),angles(2),angles(3));
tform(1:3,4) = position;

%project the points to create an image
[ image ] = points2Image(points, imageSize, cam, tform, 10,0.5);

%show the image
imshow(image);