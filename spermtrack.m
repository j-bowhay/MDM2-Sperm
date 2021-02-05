%% Track an Occluded Object
% Detect and track a ball using Kalman filtering, foreground detection, and 
% blob analysis.
% 
% Create System objects to read the video frames, detect foreground physical 
% objects, and display results.

videoReader = VideoReader('sperm.mp4');
videoPlayer = vision.VideoPlayer('Position',[100,100,500,400]);
foregroundDetector = vision.ForegroundDetector('NumTrainingFrames',10,...
                'InitialVariance',0.05);
blobAnalyzer = vision.BlobAnalysis('AreaOutputPort',false,...
                'MinimumBlobArea',70);
%% 
% Process each video frame to detect and track the ball. After reading the 
% current video frame, the example searches for the ball by using background subtraction 
% and blob analysis. When the ball is first detected, the example creates a Kalman 
% filter. The Kalman filter determines the ball?s location, whether it is detected 
% or not. If the ball is detected, the Kalman filter first predicts its state 
% at the current video frame. The filter then uses the newly detected location 
% to correct the state, producing a filtered location. If the ball is missing, 
% the Kalman filter solely relies on its previous state to predict the ball's 
% current location.
%%
  kalmanFilter = []; isTrackInitialized = false;
   while hasFrame(videoReader)
     colorImage  = readFrame(videoReader);
  
     foregroundMask = step(foregroundDetector, rgb2gray(im2single(colorImage)));
     detectedLocation = step(blobAnalyzer,foregroundMask);
     isObjectDetected = size(detectedLocation, 1) > 0;
  
     if ~isTrackInitialized
       if isObjectDetected
         kalmanFilter = configureKalmanFilter('ConstantAcceleration',...
                  detectedLocation(1,:), [1 1 1]*1e5, [25, 10, 10], 25);
         isTrackInitialized = true;
       end
       label = ''; circle = zeros(0,3);
     else 
       if isObjectDetected 
         predict(kalmanFilter);
         trackedLocation = correct(kalmanFilter, detectedLocation(1,:));
         label = 'Sperm Location';
       else
         trackedLocation = predict(kalmanFilter);
         label = 'Sperm Predicted Location';
       end
       circle = [trackedLocation, 20];
     end
  
     colorImage = insertObjectAnnotation(colorImage,'circle',...
                circle,label,'Color','red');
     step(videoPlayer,colorImage);
     pause(0.1);
   end
%% 
% Release resources.
%%
release(videoPlayer);
