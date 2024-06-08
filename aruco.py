import cv2
import pyrealsense2 as rs
import numpy as np


class ArUco:

    def __init__(self, marker_length=0.025, base_marker_id=0, object_marker_id=1):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.marker_length = marker_length
        self.camera_matrix = np.array([[593.41827166, 0, 313.63984994],
                                       [0, 593.62545055, 251.75863783],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([[0.0130745949, 0.646725640, 0.00203177405, 0.000309401928, -1.95934330]])
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.base_marker_id = base_marker_id
        self.object_marker_id = object_marker_id

    def get_marker_info(self):
        Q = np.zeros(3)
        
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is not None and self.base_marker_id in ids and self.object_marker_id in ids:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            base_idx = np.where(ids == self.base_marker_id)[0][0]
            object_idx = np.where(ids == self.object_marker_id)[0][0]

            rvec_base = rvecs[base_idx][0]
            tvec_base = tvecs[base_idx][0]
            rvec_object = rvecs[object_idx][0]
            tvec_object = tvecs[object_idx][0]

            # Get the rotation matrices
            rotation_matrix_base, _ = cv2.Rodrigues(rvec_base)
            rotation_matrix_object, _ = cv2.Rodrigues(rvec_object)

            # Compute the relative rotation matrix
            relative_rotation_matrix = np.dot(rotation_matrix_base.T, rotation_matrix_object)

            # Compute the relative translation vector
            relative_translation_vector = np.dot(rotation_matrix_base.T, tvec_object - tvec_base)

            # Compute the yaw angle from the relative rotation matrix
            yaw = np.arctan2(relative_rotation_matrix[1, 0], relative_rotation_matrix[0, 0])
            yaw_deg = np.degrees(yaw)

            # Store the relative position and yaw in Q
            Q[:] = [round((relative_translation_vector[0]-(24.3/1000)+0.0276), 4), 
                round(relative_translation_vector[1], 4), 
                round(yaw, 4)]

        cv2.imshow('Video Feed', frame)
        key = cv2.waitKey(1)
        if key == 27:  # Exit on pressing ESC key
            self.pipeline.stop()
            cv2.destroyAllWindows()

        return Q

# aruco = ArUco(marker_length=0.025, base_marker_id=0, object_marker_id=1)

# dt = 1e-2
# to = 0
# tf = 5
# m = 0.1 
# a = 0.1

# timer = np.arange(to, tf + dt, dt)
# trials = 50     

# lambda_ILC = 0.3
# gamma_ILC = 0.7

# bodyF = [np.zeros((3, len(timer) - 1)) for _ in range(trials)]
# error = [None] * trials
# G = [[None] * (len(timer) - 1) for _ in range(trials)]
# fingerF = [[np.zeros(4)] * (len(timer) - 1) for _ in range(trials)]
# tau = [[None] * (len(timer) - 1) for _ in range(trials)]

# for i in range(trials):
#     for j in range(len(timer) - 1):
#         Q = aruco.get_marker_info()
#         print(f"i: {i}, j: {j}, x: {Q[0]}, y: {Q[1]}, yaw: {Q[2]} ")
