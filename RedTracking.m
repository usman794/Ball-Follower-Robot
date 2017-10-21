function [detectedLocation,rgbFrame,isTrackInitialized,kalmanFilter] = RedTracking( rgbFrame,BW,blobAnalyzer,kalmanFilter,isTrackInitialized,velpub,velmsg)

     %kalmanFilter = [];
     %BW = zeros(size(rgbFrame(:,:,1)));
     HSV = rgb2hsv(rgbFrame);
     send(velpub,velmsg);
     H = HSV(:,:,1)*255;
     S = HSV(:,:,2)*255;
     V = HSV(:,:,3)*255;
     
     BW(:,:)=0;
      send(velpub,velmsg);
     BW_H = roicolor(H,230,255);
     BW_V = roicolor(V,50,255);
     BW_S = roicolor(S,50,255);
     send(velpub,velmsg);
     BW = BW_H & BW_V & BW_S ;
     BW = medfilt2(BW,[15 15]);
      send(velpub,velmsg);
     
     detectedLocation = step(blobAnalyzer,  BW );
     isObjectDetected = size(detectedLocation, 1) > 0;
      send(velpub,velmsg);
    if ~isTrackInitialized
     if isObjectDetected
     kalmanFilter = configureKalmanFilter('ConstantVelocity',detectedLocation(1,:), [1 1]*1e5, [25, 10], 25);
     isTrackInitialized = true;
      send(velpub,velmsg);
     end
     label = ''; circle = zeros(0,3);
    else
   if isObjectDetected
    predict(kalmanFilter);
    trackedLocation = correct(kalmanFilter, detectedLocation(1,:));
    label = 'Corrected';
     send(velpub,velmsg);
 else
 trackedLocation = predict(kalmanFilter);
 label = 'Predicted';
  send(velpub,velmsg);

 end
 circle = [trackedLocation, 5];
 send(velpub,velmsg);
 end
 rgbFrame = insertObjectAnnotation(rgbFrame,'circle',circle,label,'Color','blue');
 send(velpub,velmsg);
end

