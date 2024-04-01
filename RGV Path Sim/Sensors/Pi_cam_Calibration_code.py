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
    images = ["img1.jpg", "img2.jpg", "img3.jpg", "img4.jpg", "img5.jpg", "img6.jpg", "img7.jpg", "img8.jpg", "img9.jpg", "img10.jpg",
              "img11.jpg", "img12.jpg", "img13.jpg", "img14.jpg"]# "ss.jpg", "phone.jpg","phone_print.jpg"]
    # TODO: The list of images above should only contain pictures from the same camera, so remove ss.jpg and phone.jpg

    # Loop over your images
    for img in images:
        image = cv2.imread(img)

        (corners, ids, _) = detector.detectMarkers(image)
        ids -= np.min(ids) #Correct for possible id offsets

        if len(corners) > 0:
            charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, image, board)
            
            if charuco_retval:
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