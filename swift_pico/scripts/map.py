import cv2
import numpy as np
import cv2.aruco as aruco

class Arena:

    def __init__(self, image_path):
        self.width = 1000
        self.height = 1000
        self.image_path = image_path
        self.detected_markers = []
        self.obstacles = 0
        self.total_area = 0

    def identification(self):
        # Read the image
        frame = cv2.imread(self.image_path)

        ###################################
        # Identify the Aruco markers in the given image

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        arucoParameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=arucoParameters)

        if ids is not None:
            self.detected_markers = ids.flatten().tolist()  # Store detected marker IDs
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            # cv2.imshow('Detected Aruco Markers', frame)
            # cv2.waitKey(0)

            ###################################
            # Apply Perspective Transform

            # Extracting the Aruco marker corner points
            if len(corners) >= 4:
                # Assuming the 4 Aruco markers detected correspond to the corners of your area of interest
                # Sort them based on IDs to ensure the order is correct
                marker_corners = {id[0]: corner[0] for id, corner in zip(ids, corners)}
                sorted_ids = sorted(marker_corners.keys())

                # Define the source points (these are the corners of the Aruco markers)
                src_points = np.array([
                    marker_corners[sorted_ids[0]][0],  # Top-left
                    marker_corners[sorted_ids[1]][1],  # Top-right
                    marker_corners[sorted_ids[2]][2],  # Bottom-right
                    marker_corners[sorted_ids[3]][3]   # Bottom-left
                ], dtype="float32")

                # Define the destination points for perspective transformation (a perfect square or rectangle)
                dst_points = np.array([
                    [0, 0],                      # Top-left corner in the transformed image
                    [self.width - 1, 0],          # Top-right corner
                    [self.width - 1, self.height - 1],  # Bottom-right corner
                    [0, self.height - 1]          # Bottom-left corner
                ], dtype="float32")

                # Calculate the perspective transformation matrix
                M = cv2.getPerspectiveTransform(src_points, dst_points)

                # Apply the perspective transformation
                transformed_image = cv2.warpPerspective(frame, M, (self.width, self.height))

                # Show the transformed image
                # cv2.imshow('Transformed Image', transformed_image)
                # cv2.waitKey(0)

                ###################################
                # Use the transformed image to find obstacles (this part needs to be defined based on your needs)
                # Placeholder for obstacle detection logic (e.g., thresholding, contour detection)
                # Here, you would process `transformed_image` to identify obstacles

                # Convert the transformed image to grayscale
                gray_transformed = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)

                # Apply GaussianBlur to reduce noise and improve contour detection
                blurred = cv2.GaussianBlur(gray_transformed, (5, 5), 0)

                # Use Canny edge detection to find edges in the image
                edges = cv2.Canny(blurred, 50, 150)
                cv2.imshow('Edges', edges)

                # Find contours based on the edges detected
                contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                self.obstacles = len(contours)  # Count the obstacles (e.g., contours)
                self.total_area = sum(cv2.contourArea(c) for c in contours)  # Calculate total area of the obstacles

                # Display the obstacle detection result
                cv2.drawContours(transformed_image, contours, -1, (0, 255, 0), 2)
                cv2.imshow('Obstacles', transformed_image)
                cv2.waitKey(0)

        else:
            print("No Aruco markers detected.")

    def text_file(self):
        with open("obstacles.txt", "w") as file:
            file.write(f"Aruco ID: {self.detected_markers}\n")
            file.write(f"Obstacles: {self.obstacles}\n")
            file.write(f"Area: {self.total_area}\n")


if __name__ == '__main__':
    image_path = 'task1c_image.jpg'
    arena = Arena(image_path)
    arena.identification()
    arena.text_file()
