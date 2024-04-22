classdef constants
    properties (Constant)
        CAMERA_SOURCE = 10
        BLUETOOTH_SOURCE = 11
        CAMERA_WEIGHT = 100
        BLUETOOTH_WEIGHT = 1
        ESTIMATE_WINDOW_SIZE = 6
        ESTIMATE_HZ = 2

        ESTIMATE_GAP = 1/constants.ESTIMATE_HZ
    end
end