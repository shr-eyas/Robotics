import cv2
import numpy as np

cap = cv2.VideoCapture(0)

camera_matrix = np.array(((901.53895744, 0, 645.04484801), (0, 899.21958841, 345.56518749), (0,0,1)))
dist_coeffs = np.array((9.18995337e-02, 1.12564193e-01, -6.00012125e-03, -1.89126867e-04, -8.26495057e-01))

marker_length = 0.05

# Set coordinate system
obj_points = np.zeros((4, 1, 3), dtype=np.float32)
obj_points[0] = [-marker_length/2, marker_length/2, 0]
obj_points[1] = [marker_length/2, marker_length/2, 0]
obj_points[2] = [marker_length/2, -marker_length/2, 0]
obj_points[3] = [-marker_length/2, -marker_length/2, 0]

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters_create()

while cap.isOpened():
    
    ret, image = cap.read()
    
    image_copy = image.copy()

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image_copy, corners, ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # Calculate pose for each marker using solvePnP
        for i in range(len(ids)):
            _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], camera_matrix, dist_coeffs)

            # Transpose the rotation vector to have the shape (1, 3)
            rvecs[i] = rvec.T if rvec is not None else None
            tvecs[i] = tvec.T if tvec is not None else None

        # Draw axis for each marker using drawFrameAxes
        for i in range(len(ids)):
            
            if rvecs[i] is not None:
                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvecs[i])
                
                tvecs_reshaped = tvecs[i].reshape(3, 1)

                pose_mat = np.hstack((R, tvecs_reshaped))
                pose_mat = np.vstack((pose_mat, [0, 0, 0, 1]))

                # Draw coordinate axes on the image
                axis_len = 0.1  # Length of the axes
                img_points, _ = cv2.projectPoints(np.array([(0, 0, 0), (axis_len, 0, 0), (0, axis_len, 0), (0, 0, axis_len)]), rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
                img_points = img_points.astype(int)

                # Draw the axes lines
                cv2.line(image_copy, tuple(img_points[0].ravel()), tuple(img_points[1].ravel()), (0, 0, 255), 2)  # x-axis (red)
                cv2.line(image_copy, tuple(img_points[0].ravel()), tuple(img_points[2].ravel()), (0, 255, 0), 2)  # y-axis (green)
                cv2.line(image_copy, tuple(img_points[0].ravel()), tuple(img_points[3].ravel()), (255, 0, 0), 2)  # z-axis (blue)


    # Show resulting image and close window
    cv2.imshow("out", image_copy)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
