% Calculates the windowed unbiased 2DRMS accuracy for the specified
% estimates.
% Windowed = Each 2DRMS calulation only considers estimates from some
% specific window of time.
% Unbiased = This assumes that the mean of the estimates is the true RGV
% location. Essentially this uses standard deviation instead of standard
% error in the 2DRMS equation.
function twodrms = calculate_windowed_unbiased_2drms(estimates, times, window_size)
    twodrms = 2*sqrt(sum(movstd(estimates, window_size, "SamplePoints", times).^2, 2));
end