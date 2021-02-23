close all;
videoReader = VideoReader('Rabbit-3.avi');
videoPlayer = vision.VideoPlayer;

Height = videoReader.Height;
FirstFrame = readFrame(videoReader);
EdgeFirstFrame = TurnEdge(FirstFrame);
% imshow(FirstFrame)
% LinePoint = ginput(2);
% X = LinePoint(1,:);
% Y = LinePoint(2,:);
% m = (Y(2)-Y(1))/(X(2)-X(1));
% C = -m*X(1)+Y(1);
% y = @(x) m*x+C;
% x1 = 0; y1 = y(x1);
% x2 = Height; y2 = y(x2);
% LineFrame = insertShape(FirstFrame,'Line',[x1 y1 x2 y2],'Color','white');
% hold on;
% imshow(LineFrame);

% detector = vision.ForegroundDetector('NumTrainingFrames', 3,...
%     'InitialVariance', 50, 'MinimumBackgroundRatio', 0.6);
% blob = vision.BlobAnalysis('CentroidOutputPort', false,...
%     'AreaOutputPort', false, 'BoundingBoxOutputPort', true,...
%     'MinimumBlobAreaSource', 'Property', 'MinimumBlobArea', 250);
% shapeInserter = vision.ShapeInserter('BorderColor','White');
% while hasFrame(videoReader)
%     frame = readFrame(videoReader);
%     fgMask = detector(frame);
%     bbox = blob(fgMask);
%     out = shapeInserter(frame,bbox);
%     videoPlayer(out);
%     pause(0.1);
% end

 
InitPoint = [103.905940594059,126.222772277228];
% InitPoint = [133.569306930693,188.044554455446];
figure;
imshow(EdgeFirstFrame)
% InitPoint = ginput(1);
% MarkerFrame = insertMarker(FirstFrame,InitPoint,'+','Color','White');
HeadRect = drawrectangle();
HeadROI = HeadRect.Position;
MarkerFrame = insertShape(FirstFrame,'Rectangle',HeadROI);
RFrame = rotateFrames(MarkerFrame,HeadROI,InitPoint);
imshow(MarkerFrame);
figure;
imshow(RFrame);
% imshow(MarkerFrame)

% pointTracker = vision.PointTracker('MaxIterations', 100);
% initialize(pointTracker,point,FirstFrame);
% while hasFrame(videoReader)
%     frame = readFrame(videoReader);
%     [points,validity] = pointTracker(frame);
%     out = insertMarker(frame,points(validity,:),'+');
%     videoPlayer(out);
%     pause(0.1);
% end
% release(videoPlayer);

NotTop = true;
point = int64(InitPoint);
% XList = [InitPoint(1)];
% YList = [InitPoint(2)];
HeadAngleList = [];
while hasFrame(videoReader)
    OriginalFrame = readFrame(videoReader);
    frame = TurnEdge(OriginalFrame); 
%     while NotTop
%         range = 1;
%         RMin = int64(point(2)-range);
%         RMax = int64(point(2)+range);
%         CMin = int64(point(1)-range);
%         CMax = int64(point(1)+range);
%         ROI = frame(RMin:RMax,CMin:CMax);
%         if ROI(1,1)==0 && ROI(1,2)==0 && ROI(1,3)==0
%             NotTop = false;
%         elseif ROI(1,1)==1 && ROI(1,2)==0 && ROI(1,3)==0
%             TopPoint(1) = point(1)-1;   %col
%             TopPoint(2) = point(2)-1;   %row
%         elseif ROI(1,1)==0 && ROI(1,2)==1 && ROI(1,3)==0
%             TopPoint(1) = point(1);     %col
%             TopPoint(2) = point(2)-1;   %row
%         elseif ROI(1,1)==0 && ROI(1,2)==0 && ROI(1,3)==1    
%             TopPoint(1) = point(1)+1;   %col
%             TopPoint(2) = point(2)-1;   %row
%         elseif ROI(1,1)==1 && ROI(1,2)==1 && ROI(1,3)==0 ||...
%                 ROI(1,1)==0 && ROI(1,2)==1 && ROI(1,3)==1 ||...
%                 ROI(1,1)==1 && ROI(1,2)==1 && ROI(1,3)==1
%             TopPoint(1) = point(1);     %col
%             TopPoint(2) = point(2)-1;   %row
%         else
%             [r, c] = find(ROI==1);
%             row = min(r);
%             pos = find(r==row,1,'first');
%             col = c(pos);
%             CMin = double(CMin); RMin = double(RMin);
%             TopPoint = [CMin+col-1,RMin+row-1];
%         end
%     end
    for i=1:2
        RMin = int64(point(2)-i);
        RMax = int64(point(2)+i);
        CMin = int64(point(1)-i);
        CMax = int64(point(1)+i);
        ROI = frame(RMin:RMax,CMin:CMax);
        [r, c] = find(ROI==1);
        if isempty(r)||isempty(c)
            continue
        elseif size(r)>[1 0]
            row = min(r);
            pos = find(r==row,1,'first');
            col = c(pos);
            CMin = double(CMin); RMin = double(RMin);
            point = [CMin+col-1,RMin+row-1];
%             out = insertMarker(OriginalFrame,point,'+');
            break
        else
            row = r;
            col = c;
            CMin = double(CMin); RMin = double(RMin);
            point = [CMin+col-1,RMin+row-1];
%             out = insertMarker(OriginalFrame,point,'+');
            break
        end
    end
    
    ROI = [point(1)-10 point(2)-5 28 30];
%     out = insertShape(OriginalFrame,'Rectangle',ROI);
    [RotatedOut,HeadAngle] = rotateFrames(OriginalFrame,ROI,point);
    HeadAngleList(end+1,1)=HeadAngle;
    videoPlayer(RotatedOut);
%     disp(theta);
%     XList(end+1) = point(1);
%     YList(end+1) = point(2);

%     pause(0.1);
end

writematrix(HeadAngleList,'HeadAngleList.txt','WriteMode','overwrite');
% figure;
% plot(XList,YList)
function EdgeFrame = TurnEdge(Frame)
GrayFrame = rgb2gray(Frame);
EnhancedFrame = adapthisteq(GrayFrame);
FilteredFrame = filter2(fspecial('average',3),EnhancedFrame);
EdgeFrame = edge(FilteredFrame);
EdgeFrame = double(EdgeFrame);
end

function [RotatedFrame,theta] = rotateFrames(OriginalFrame,ROI,MarkerPoint)
CroppedFrame = imcrop(OriginalFrame,ROI);
GrayCroppedFrame = rgb2gray(CroppedFrame);
FCroppedFrame = double(GrayCroppedFrame);
BWCroppedFrame = imbinarize(CroppedFrame);
BWCroppedFrame = double(BWCroppedFrame);
% imshow(BWCroppedFrame);
Min = min(size(FCroppedFrame));
ReshapedFrame = FCroppedFrame(1:Min,1:Min);
CoorList=[];
i = 1;
for x=1:Min
    for y=1:Min
        if BWCroppedFrame(y,x)==1
            CoorList(i,:)=[x,y];
            i = i+1;
        else
            continue
        end
        
    end
end
Cov = cov(CoorList);
[EVec,EVal] = eig(Cov);
[MaxEVal,~] = max(EVal);
% MaxEVal = max(EVal);
% [MaxR,MaxC] = find(EVal==max(MaxEVal));
[~,p] = max(MaxEVal);
v=EVec(:,p);
theta=-atan(v(1)/v(2));
% M = [EVec,EVec,theta];
% writematrix(M,'M.txt','Delimiter','tab','WriteMode','append');
% disp(EVal);
% disp(EVec);
% disp(v);
% disp(theta);
OriginalFrame = insertMarker(OriginalFrame,MarkerPoint,'+');
% RotatedFrame = imrotate(OriginalFrame,rad2deg(-theta),'crop');
RotatedFrame = rotateAround(OriginalFrame,MarkerPoint(1),MarkerPoint(2),rad2deg(theta),'bicubic');
RotatedFrame = imfuse(RotatedFrame,BWCroppedFrame,'blend','Scaling','joint');
end


