% export ROS_MASTER_URI=http://turtlebot4:11311
% rosrun rqt_reconfigure rqt_reconfigure

rosshutdown
clear all
clc
rosinit('http://turtlebot6:11311')
threshold = 100;
threshod1 = 0.1;
kp=0.003;
kp1=0.1;
y_o=320;
z_o=0.5;

handles.colorImgSub = exampleHelperTurtleBotEnableColorCamera;
handles.cliffSub = rossubscriber('/mobile_base/events/cliff','BufferSize',5);
handles.bumpSub= rossubscriber('/mobile_base/sensors/bumper_pointcloud','BufferSize',5);
handles.soundSub= rospublisher('/mobile_base/commands/sound','kobuki_msgs/Sound');
handles.velpub=rospublisher('mobile_base/commands/velocity');
handles.camInfo=rossubscriber('/camera/rgb/image_color','sensor_msgs/CameraInfo');
velpub=rospublisher('mobile_base/commands/velocity');
ptclSub = rossubscriber('/camera/depth_registered/image');
velmsg=rosmessage(velpub);

blobAnalyzer = vision.BlobAnalysis;
blobAnalyzer.MinimumBlobArea = 1000;
blobAnalyzer.AreaOutputPort = false;
blobAnalyzer.MaximumCount = 1;
nFrame = 0; 
kalmanFilter = []; isTrackInitialized = false; 
rgbFrame= readImage(handles.colorImgSub.LatestMessage);
BW = zeros(size(rgbFrame(:,:,1)));
x=0;
z=0;
velmsg.Angular.Z=0;
velmsg.Linear.X=0;


while(1)
%    tic;
    send(velpub,velmsg);
     rgbFrame= readImage(handles.colorImgSub.LatestMessage);
%      BW = zeros(size(rgbFrame(:,:,1)));
%      HSV = rgb2hsv(rgbFrame);
%      H = HSV(:,:,1)*255;
%      S = HSV(:,:,2)*255;
%      V = HSV(:,:,3)*255;
%      
%      BW(:,:)=0;
%      BW_H = roicolor(H,230,255);
%      BW_V = roicolor(V,50,255);
%      BW_S = roicolor(S,50,255);
%      BW = BW_H & BW_V & BW_S ;
%      BW = medfilt2(BW,[15 15]);
%      
%      detectedLocation = step(blobAnalyzer,  BW );
%      isObjectDetected = size(detectedLocation, 1) > 0;
%      send(velpub,velmsg);
%     if ~isTrackInitialized
%      if isObjectDetected
%     kalmanFilter = configureKalmanFilter('ConstantVelocity',detectedLocation(1,:), [1 1]*1e5, [25, 10], 25);
%     isTrackInitialized = true;
%      end
%      label = ''; circle = zeros(0,3);
%     else
%   if isObjectDetected
%     predict(kalmanFilter);
%     trackedLocation = correct(kalmanFilter, detectedLocation(1,:));
%     label = 'Corrected';
%  else
%  trackedLocation = predict(kalmanFilter);
%  label = 'Predicted';
% 
%  end
%  circle = [trackedLocation, 5];
%  end
%  rgbFrame = insertObjectAnnotation(rgbFrame,'circle',circle,label,'Color','blue');
 [detectedLocation,rgbFrame,isTrackInitialized,kalmanFilter] = RedTracking(rgbFrame,BW,blobAnalyzer,kalmanFilter,isTrackInitialized,velpub,velmsg);
 nFrame = nFrame+1;
 send(velpub,velmsg);
 %imshow(rgbFrame);
 %figure
 imshow(BW)
 display(detectedLocation );
 send(velpub,velmsg);

% if isempty(detectedLocation)
%             for i=0 : 1500
%                 velmsg.Linear.X=0; 
%                 velmsg.Angular.Z=0.5;
%                 send(velpub,velmsg);
%                 if ~isempty(detectedLocot. Turn on thation)
%                     break;
%                 end
%             end
% end
while(isempty(detectedLocation))
       velmsg.Linear.X=0; 
       velmsg.Angular.Z=0.55;
       send(velpub,velmsg);
       rgbFrame= readImage(handles.colorImgSub.LatestMessage);
       send(velpub,velmsg);
       [detectedLocation,rgbFrame,isTrackInitialized,kalmanFilter] = RedTracking(rgbFrame,BW,blobAnalyzer,kalmanFilter,isTrackInitialized,velpub,velmsg);
       if ~isempty(detectedLocation)
           break;
       end
end     
send(velpub,velmsg);
% delta_t=toc;           
if ~isempty(detectedLocation)
    
        ptclmsg = receive(ptclSub);
        send(velpub,velmsg);
        depth_image = readImage(ptclmsg);
        send(velpub,velmsg);
        depth_centroid = depth_image(floor(detectedLocation(1,2)),floor(detectedLocation(1,1)));
        x=detectedLocation(1,2);
        y=detectedLocation(1,1);
        z=depth_centroid;
        send(velpub,velmsg);
    
        if (abs(y-y_o) > threshold && abs(z-z_o) > threshod1)   
            velmsg.Angular.Z=-kp*(y-y_o);
            velmsg.Linear.X=kp1*(z-z_o);
            send(velpub,velmsg);
      
        elseif (abs(y-y_o) > threshold && abs(z-z_o) <= threshod1)
            velmsg.Angular.Z=-kp*(y-y_o);
            velmsg.Linear.X=0
            send(velpub,velmsg);
          
            
        elseif (abs(y-y_o) <= threshold && abs(z-z_o) > threshod1)
            velmsg.Angular.Z=0;
            velmsg.Linear.X=kp1*(z-z_o);
            send(velpub,velmsg);
        
            
        else
            velmsg.Angular.Z=0;
            velmsg.Linear.X=0
            send(velpub,velmsg);
        end
    
end 
end
