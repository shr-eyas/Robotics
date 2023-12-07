import cv2
import numpy as np

class ArUcoPoseEstimation:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.camera_matrix = np.array(((901.53895744, 0, 645.04484801), (0, 899.21958841, 345.56518749), (0, 0, 1)))
        self.dist_coeffs = np.array((9.18995337e-02, 1.12564193e-01, -6.00012125e-03, -1.89126867e-04, -8.26495057e-01))
        self.marker_length = 25 / 1000
        self.obj_points = np.zeros((4, 1, 3), dtype=np.float32)
        self.obj_points[0] = [-self.marker_length / 2, self.marker_length / 2, 0]
        self.obj_points[1] = [self.marker_length / 2, self.marker_length / 2, 0]
        self.obj_points[2] = [self.marker_length / 2, -self.marker_length / 2, 0]
        self.obj_points[3] = [-self.marker_length / 2, -self.marker_length / 2, 0]
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def detect_and_draw_markers(self, image):
        image_copy = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image_copy, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix,
                                                                  self.dist_coeffs)

            for i in range(len(ids)):
                _, rvec, tvec = cv2.solvePnP(self.obj_points, corners[i], self.camera_matrix, self.dist_coeffs)
                rvecs[i] = rvec.T if rvec is not None else None
                tvecs[i] = tvec.T if tvec is not None else None

            self.draw_axes(image_copy, ids, rvecs, tvecs)

        return image_copy

    def draw_axes(self, image, ids, rvecs, tvecs):
        for i in range(len(ids)):
            if rvecs[i] is not None:
                R, _ = cv2.Rodrigues(rvecs[i])
                tvecs_reshaped = tvecs[i].reshape(3, 1)
                pose_mat = np.hstack((R, tvecs_reshaped))
                pose_mat = np.vstack((pose_mat, [0, 0, 0, 1]))

                axis_len = 0.1
                img_points, _ = cv2.projectPoints(
                    np.array([(0, 0, 0), (axis_len, 0, 0), (0, axis_len, 0), (0, 0, axis_len)]), rvecs[i], tvecs[i],
                    self.camera_matrix, self.dist_coeffs)
                img_points = img_points.astype(int)

                cv2.line(image, tuple(img_points[0].ravel()), tuple(img_points[1].ravel()), (0, 0, 255), 2)  # x-axis
                cv2.line(image, tuple(img_points[0].ravel()), tuple(img_points[2].ravel()), (0, 255, 0), 2)  # y-axis
                cv2.line(image, tuple(img_points[0].ravel()), tuple(img_points[3].ravel()), (255, 0, 0), 2)  # z-axis

    def run(self):
        while self.cap.isOpened():
            ret, image = self.cap.read()
            image_with_markers = self.detect_and_draw_markers(image)
            cv2.imshow("out", image_with_markers)

            key = cv2.waitKey(1)
            if key == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    aruco_pose_estimator = ArUcoPoseEstimation()
    aruco_pose_estimator.run()
