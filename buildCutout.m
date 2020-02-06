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

xyz = zeros(sensorWidth*sensorHeight, 3);
color = zeros(sensorWidth*sensorHeight, 3);

for x=-sensorWidth/2+1:sensorWidth/2
    for y=-sensorHeight/2+1:sensorHeight/2
        to = from + f * scaling * cameraDirection + x * scaling * sensorX + y * scaling * sensorY;
        xyz(i,:) = to;
        color(i,:) = [255 255 255];
        i = i + 1;
    end
end

xyz(i,:) = from;
color(i,:) = [255 255 255];

pc = pointCloud(xyz, 'Color', color);
pcwrite(pc, 'camera_sensor.ply');