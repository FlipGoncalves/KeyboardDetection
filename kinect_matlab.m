clear

diskLogger = VideoWriter("VideoColor.avi");
diskLogger.FrameRate = 30; % Sets the framerate of the recorded and saved avi video file

% Create the VIDEOINPUT objects for the two streams
colorVid = imaq.VideoDevice('kinect',1);
depthVid = imaq.VideoDevice('kinect',2);

open(diskLogger);

step(colorVid);
step(depthVid);

color = uint8(zeros(480,640,3,100));
depth = uint16(zeros(480,640,100));

a = 1

for i=1:100
    color(:,:,:,i) = step(colorVid);
    depth(:,:,i) = step(depthVid);
end

b = 186129476796318273916

% Stop the devices
release(colorVid);
release(depthVid);

for i=1:100
    ptCloud = pcfromkinect(depthVid,depth(:,:,i), color(:,:,:,i));
    pcwrite(ptCloud, sprintf('pointclouds/object3d%d.pcd',i),'Encoding','ascii');
end

writeVideo(diskLogger,color);
% implay(colorFrameData);

close(diskLogger);