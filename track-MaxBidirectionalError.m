close all;
videoReader = VideoReader('Rabbit-3.avi');
videoPlayer = vision.VideoPlayer;
depVideoPlayer = vision.DeployableVideoPlayer;

objectFrame = readFrame(videoReader);
objectRegion = [97 125 20 24]; %[97 125 20 24]-Rabbit-3 %[17 135 8 30]
objectImage = insertShape(objectFrame,'Rectangle',objectRegion,...
    'Color','red');
% figure;
% imshow(objectImage);

points = detectMinEigenFeatures(rgb2gray(objectFrame),'ROI',objectRegion);
pointImage = insertMarker(objectFrame,points.Location,'+','Color','White');
% figure;
% imshow(pointImage);

tracker = vision.PointTracker('MaxBidirectionalError',Inf);
initialize(tracker,points.Location,objectFrame);

while hasFrame(videoReader)
    frame = readFrame(videoReader);
    [points,validity] = tracker(frame);
    out = insertMarker(frame,points(validity,:),'+');
%     imshow(out)
%     videoPlayer(out);
    depVideoPlayer(out);
%     step(videoPlayer,out);
end

% shg;
