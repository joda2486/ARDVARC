function cv_direction_vectors = cv_post(bag)
    camera = select(bag, "Topic", "usb_cam/image_raw/compressed");
    camera_msgs = cell2mat(readMessages(camera,'DataFormat','struct'));
    headers = [camera_msgs.Header];
    stamps = [headers.Stamp];
    camera_times = double([stamps.Sec]') + 1e-9 * double([stamps.Nsec]') - bag.StartTime;
    intrinsics = cameraIntrinsicsFromOpenCV([914.0147243, 0, 640; 0, 914.0147243, 400; 0, 0, 1], zeros(1,5), [800 1280]);
    marker_size = .352;
    extrinsics_matrix = eul2rotm(deg2rad([90, 180, 180]), "ZYZ");
    extrinsics_tvec = [0.2, 0, -0.0889];
    
    Direction = [];
    Time = [];
    RgvId = [];
    
    tic
    for i = 1:length(camera_msgs)
        camera_matrix = rosReadImage(camera_msgs(i));
    
        [id,~,pose] = readAprilTag(camera_matrix, "DICT_4X4_50",intrinsics,marker_size);
        for idx = 1:length(id)
            if (id ~= 4)
                disp("Got weird aruco id? : " + id)
            end
            Time(end+1) = camera_times(i);
            Direction(end+1,:) = pose(idx).Translation;
            RgvId(end+1) = 2;
        end
    
        [id,~,pose] = readArucoMarker(camera_matrix, "DICT_4X4_50",intrinsics,marker_size);
        for idx = 1:length(id)
            if (id ~= 5)
                disp("Got weird aruco id? : " + id)
            end
            Time(end+1) = camera_times(i);
            Direction(end+1,:) = pose(idx).Translation;
            RgvId(end+1) = 1;
        end
    end
    toc
    
    Direction = permute(pagemtimes(extrinsics_matrix,permute(Direction,[2,3,1])),[3,1,2]);
    Direction = Direction + extrinsics_tvec;
    
    MeasurementSource = ones(length(Time), 1)*constants.CAMERA_SOURCE;
    Time = Time';
    RgvId = RgvId';
    
    cv_direction_vectors = table(Time,Direction,RgvId,MeasurementSource);
end