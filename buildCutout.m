f = 500;
sensorSize = [1000,1000];
rFix = [0.0, 0.0, 180.0];
r = rFix + sweepRecord.rotation;
fov = 2*atan((sensorSize(1)/2)/f);
angles = r*pi/180;
R = angle2dcm(angles(1),angles(2),angles(3));
cameraCoordinateSystem = R * eye(3);

%%

from = sweepRecord.position';

sensorWidth = sensorSize(1);
sensorHeight = sensorSize(2);
sensorX = cameraCoordinateSystem(1,:);
sensorY = -cameraCoordinateSystem(2,:);
cameraDirection = cameraCoordinateSystem(3,:);

%%

i = 1;
scaling = 1/f;

sensorPoints = zeros(sensorWidth*sensorHeight, 3);
color = zeros(sensorWidth*sensorHeight, 3);

for x=-sensorWidth/2+1:sensorWidth/2
    for y=-sensorHeight/2+1:sensorHeight/2
        to = from + f * scaling * cameraDirection + x * scaling * sensorX + y * scaling * sensorY;
        sensorPoints(i,:) = to;
        color(i,:) = [255 255 255];
        i = i + 1;
    end
end

%sensorPoints(i,:) = from;
%color(i,:) = [255 255 255];

pc2 = pointCloud(sensorPoints, 'Color', color);
pcwrite(pc2, 'camera_sensor.ply');

pc = pcread('matterpak_sehs6V3VnSW/cloud - rotated.ply');

%%
% % PROBLEM: This code is to be executed 1000x1000 times!
% pointLineDist = zeros(size(xyz,1), 1);
% denominator = norm((to-from));
% for i=1:size(pc.Location,1)
%     point = pc.Location(i,:);
%     pointLineDist(i) = norm(cross((point-from), (point - to)))/denominator;
% end
% nearThresh = 0.01;
% hit = find(pointLineDist < nearThresh);
% % TODO: filter point only in front of the camera
% % TODO: find the nearest point to the camera

[tri,pts] = plyread('matterpak_sehs6V3VnSW/mesh-rotated.ply');

T = cell2mat(tri.face.vertex_indices);
T = T + 1; % fix the problem with some index (indices) are zero
trisurf(T, tri.vertex.x, tri.vertex.y, tri.vertex.z);

%%
nTriangles = size(tri.face.vertex_indices,1);
vert0 = zeros(nTriangles, 3);
vert1 = zeros(nTriangles, 3);
vert2 = zeros(nTriangles, 3);
for i=1:nTriangles
    ind = T(i,:);
    vert0(i,:) = [tri.vertex.x(ind(1)) tri.vertex.y(ind(1)) tri.vertex.z(ind(1))];
    vert1(i,:) = [tri.vertex.x(ind(2)) tri.vertex.y(ind(2)) tri.vertex.z(ind(2))];
    vert2(i,:) = [tri.vertex.x(ind(3)) tri.vertex.y(ind(3)) tri.vertex.z(ind(3))];
end

depthImage = zeros(sensorHeight, sensorWidth);

%%
for x=1:sensorWidth
    for y=1:sensorHeight
        sprintf('processing ray at sensor position (%d, %d)', x, y)
        origin = from;
        dir = sensorPoints(i,:) - origin;
        dir = dir / norm(dir);
        [intersect, t, u, v, xcoor] = TriangleRayIntersection(origin, dir, vert0, vert1, vert2);

        depths = t(intersect);
        xyzs = xcoor(intersect,:);
        [sorted, indices] = sort(depths);
        bestIdx = indices(1);
        depth = depths(bestIdx);
        xyz = xyzs(bestIdx,:); % TODO: use it
        depthImage(x,y) = depth;
    end
end