% Calculates projections from pointing vectors from a certain measurement
% source. The specified delay is added to each pointing vector. Then the
% UAS poses are interpolated to find the approximate UAS pose at the time
% that the pointing vector was recorded. The pointing vector is next
% converted to the inertial frame. Finally, the inertial pointing vector is
% followed until it intersects the ground, giving the projection location.
function [times, projections] = calculate_projections(bag, poses, delay, measurement_source, rgv_id)
    direction_vectors_select = select(bag, "Topic", "/estimation/direction_vectors_uas");
    direction_vectors_msgs = cell2mat(readMessages(direction_vectors_select,'DataFormat','struct'));
    direction_vectors = [direction_vectors_msgs.Direction]';
    direction_vector_time_structs = [direction_vectors_msgs.Timestamp];
    times = double([direction_vector_time_structs.Sec]) + 1e-9 * double([direction_vector_time_structs.Nsec]) - delay;
    mask = [direction_vectors_msgs.MeasurementSource] == measurement_source & [direction_vectors_msgs.RgvId] == rgv_id & times >= poses.times(1) & times <= poses.times(end);
    direction_vectors = direction_vectors(mask,:);
    times = times(mask);

    whole_factors = interp1(poses.times', 1:length(poses.times), times');
    p_quaternions = poses.quaternions(floor(whole_factors),:);
    q_quaternions = poses.quaternions(ceil(whole_factors),:);
    inner_factors = mod(whole_factors,1);
    warning("off","aerospace:quatlog:notUnitQuaternion")
    aligned_quaternions = quatinterp(p_quaternions, q_quaternions, inner_factors);
    aligned_rotms = quat2rotm(aligned_quaternions);
    aligned_direction_vectors_inertial = permute(pagemtimes(aligned_rotms, permute(direction_vectors, [2,3,1])), [3,1,2]);
    aligned_positions = interp1(poses.times', poses.positions', times');
    coeffs = aligned_positions(:,3) ./ aligned_direction_vectors_inertial(:,3);
    projections = aligned_positions - coeffs .* aligned_direction_vectors_inertial;

    [times, indicies] = sort(times);
    projections = projections(indicies,1:2);
end