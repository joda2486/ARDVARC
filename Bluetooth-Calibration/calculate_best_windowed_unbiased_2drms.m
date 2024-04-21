function min2drms = calculate_best_windowed_unbiased_2drms(poses, direction_vectors, delay, measurement_source, window_size)
    projections_rgv1 = calculate_projections(poses, direction_vectors, delay, measurement_source, 1);
    min2drms_rgv1 = min(calculate_windowed_unbiased_2drms(projections_rgv1.Position, projections_rgv1.Time, window_size));
    projections_rgv2 = calculate_projections(poses, direction_vectors, delay, measurement_source, 2);
    min2drms_rgv2 = min(calculate_windowed_unbiased_2drms(projections_rgv2.Position, projections_rgv2.Time, window_size));
    min2drms = min(min2drms_rgv1, min2drms_rgv2);
end