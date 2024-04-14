import numpy as np
import cv2
import cv2.aruco as aruco


def calibrate_cam():
    # Define the dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((11, 8), .015, .011, aruco_dict)
    
    board.setLegacyPattern(True)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    # Arrays to store object points and image points from all images
    all_corners = []
    all_ids = []
    images = [f"i{i}.jpg" for i in range(1, 17)]
    # TODO: The list of images above should only contain pictures from the same camera, so remove ss.jpg and phone.jpg

    # Loop over your images
    for img in images:
        image = cv2.imread(img)

        (corners, ids, _) = detector.detectMarkers(image)
        if ids is not None:
            ids -= np.min(ids) #Correct for possible id offsets
            

        if len(corners) > 0:
            charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, image, board)
            
            if charuco_retval and len(charuco_corners) >= 4:
                print("Interpolated!")
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, (0, 255, 0))
                cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 0, 255))
                cv2.imshow('Charuco Corners', image)
                cv2.waitKey(0)
            else:
                print("Failed to interpolate")
    
    # Calibrate the camera

    retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_corners, all_ids, board, image.shape[:2], None, None)
    if camera_matrix is not None:
        print("Calibration successful!\n")
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
    # Save calibration data
    np.save('camera_matrix.npy', camera_matrix)
    np.save('dist_coeffs.npy', dist_coeffs)

    # Iterate through displaying all the images
    for image_file in images:
        image = cv2.imread(image_file)
        undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs)
        cv2.imshow('Undistorted Image', undistorted_image)
        cv2.waitKey(0)


calibrate_cam()