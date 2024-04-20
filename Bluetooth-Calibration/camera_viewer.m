% Shows the camera frames from the speificed bag. Also re-runs MATLABs
% april/aruco tag detection. Finally shows a graph comparing real-time cv
% rates to post-processing cv rates.

[file, location] = uigetfile("*.bag");
bag = rosbag([location, file]);
camera = select(bag, "Topic", "usb_cam/image_raw/compressed");
camera_msgs = cell2mat(readMessages(camera,'DataFormat','struct'));
camera_ts = timeseries(camera, "Header.Seq");

frame_count = 1:length(camera_msgs);
post_seen = zeros(1,length(camera_msgs));
disp("Starting...")
for i = 1:length(camera_msgs)
    camera_matrix = rosReadImage(camera_msgs(i));
    show_matrix = camera_matrix;

    [id,loc] = readAprilTag(camera_matrix, "tag36h11");
    for idx = 1:length(id)
        markerRadius = 8;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        show_matrix = insertShape(show_matrix,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
        post_seen(i) = post_seen(i) + 1;
        disp("April " + id(idx))
    end

    [id,loc] = readArucoMarker(camera_matrix, "DICT_6X6_50");
    for idx = 1:length(id)
        markerRadius = 8;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        show_matrix = insertShape(show_matrix,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
        post_seen(i) = post_seen(i) + 1;
        disp("Aruco " + id(idx))
    end

    imshow(show_matrix)
end
post_sightings_count = cumsum(post_seen);


sightings = select(bag, "Topic", "/camera/recent_rgv_sightings");
sightings_ts = timeseries(sightings, "RgvId");
live_sightings_rgv1 = bitand(sightings_ts.Data,1);
live_sightings_rgv2 = bitand(sightings_ts.Data,2) / 2;
live_sightings_count = cumsum(live_sightings_rgv1 + live_sightings_rgv2);


figure
hold on
grid on
grid minor
plot(camera_ts.Time - camera_ts.Time(1), frame_count', DisplayName="Frames Recorded")
plot(camera_ts.Time - camera_ts.Time(1), post_sightings_count', DisplayName="Detections In Post")
plot(sightings_ts.Time - sightings_ts.Time(1), live_sightings_count, DisplayName="Real-Time Detections")
legend(Location="best")
xlabel("Time Since Start [s]")
ylabel("Occurences (Cumulative)")