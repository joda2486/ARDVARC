function truth_2drms = calculate_best_windowed_2drms_truth(estimates, estimates_times ,truth, truth_times)
% Calculates the windowed 2DRMS accuracy for the specified
% estimates.
% Windowed = Each 2DRMS calulation only considers estimates from some
% specific window of time.
truth_interp = interp1(truth_times,truth,estimates_times);

dev_E = sqrt((sum((estimates(:,1) - truth_interp(:,1)).^2))/length(estimates(:,1)));
dev_N = sqrt((sum((estimates(:,2) - truth_interp(:,2)).^2))/length(estimates(:,2)));

truth_2drms = 2 * sqrt(dev_E^2 + dev_N^2);

end