% Calculates projections from pointing vectors from a certain measurement
% source. The specified delay is added to each pointing vector. Then the
% UAS poses are interpolated to find the approximate UAS pose at the time
% that the pointing vector was recorded. The pointing vector is next
% converted to the inertial frame. Finally, the inertial pointing vector is
% followed until it intersects the ground, giving the projection location.
function t = calculate_projections(poses, direction_vectors, delay, measurement_source, rgv_id)
    direction_vectors.Time = direction_vectors.Time - delay;
    mask = direction_vectors.MeasurementSource == measurement_source & direction_vectors.RgvId == rgv_id & direction_vectors.Time >= poses.Time(1) & direction_vectors.Time <= poses.Time(end);
    direction_vectors = direction_vectors(mask,:);

    whole_factors = interp1(poses.Time', 1:length(poses.Time), direction_vectors.Time');
    p_quaternions = poses.Orientation(floor(whole_factors),:);
    q_quaternions = poses.Orientation(ceil(whole_factors),:);
    inner_factors = mod(whole_factors,1);
    warning("off","aerospace:quatlog:notUnitQuaternion")
    aligned_quaternions = quatinterp(p_quaternions, q_quaternions, inner_factors);
    aligned_rotms = quat2rotm(aligned_quaternions);
    aligned_direction_vectors_inertial = permute(pagemtimes(aligned_rotms, permute(direction_vectors.Direction, [2,3,1])), [3,1,2]);
    aligned_positions = interp1(poses.Time', poses.Position, direction_vectors.Time');
    coeffs = aligned_positions(:,3) ./ aligned_direction_vectors_inertial(:,3);
    Position = aligned_positions - coeffs .* aligned_direction_vectors_inertial;

    [Time, indicies] = sort(direction_vectors.Time);
    Position = Position(indicies,1:2);
    RgvId = direction_vectors.RgvId;
    MeasurementSource = direction_vectors.MeasurementSource;
    t = table(Time,Position,RgvId,MeasurementSource);
end