function estimate = calculate_estimate(bluetooth_projections, camera_projections, target_time, window_size)
    bluetooth_mask = bluetooth_projections.Time >= target_time - window_size/2 & bluetooth_projections.Time <= target_time + window_size/2;
    bluetooth_projections = bluetooth_projections(bluetooth_mask,:);
    camera_mask = camera_projections.Time >= target_time - window_size/2 & camera_projections.Time <= target_time + window_size/2;
    camera_projections = camera_projections(camera_mask,:);
    all_projections = [bluetooth_projections;camera_projections];
    cam_projections_mask = all_projections.MeasurementSource == constants.CAMERA_SOURCE;
    
    ft = fittype("poly1");

    if height(all_projections) < 2
        estimate = NaN(1,2);
        return
    end
    
    xfit = fit(all_projections.Time,all_projections.Position(:,1), ft, "Weight", cam_projections_mask*constants.CAMERA_WEIGHT+~cam_projections_mask*constants.BLUETOOTH_WEIGHT);
    yfit = fit(all_projections.Time,all_projections.Position(:,2), ft, "Weight", cam_projections_mask*constants.CAMERA_WEIGHT+~cam_projections_mask*constants.BLUETOOTH_WEIGHT);
    xbest = feval(xfit, target_time);
    ybest = feval(yfit, target_time);
    estimate = [xbest, ybest];
end