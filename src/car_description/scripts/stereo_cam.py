#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import  Image
import os

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

#get package path
data_folder_calib = os.path.join(get_package_share_directory('car_description'), 'data_road/training/calib/')
print(os.path.exists(data_folder_calib))
cat = ['uu', 'uum', 'um']

IDX_LEN = 6

idx_num = 1
cat_idx = 2
fname = cat[cat_idx
]+'_'+str(idx_num).zfill(IDX_LEN)
img_fname = fname + '.png'
calib_fname = fname + '.txt'


def plain(image, num):
    cv2.imshow(f"image_{num}", image)
    cv2.waitKey(1)
    return image.shape

def points_reconstruction(img_left_color,  img_left_bw, img_right_bw):
    stereo = cv2.StereoBM_create(numDisparities=96, blockSize=11)
    disparity = stereo.compute(img_left_bw,img_right_bw)

    img = disparity.copy()


    # Reading calibration
    matrix_type_1 = 'P2'
    matrix_type_2 = 'P3'

    calib_file = data_folder_calib + calib_fname
    with open(calib_file, 'r') as f:
        fin = f.readlines()
        for line in fin:
            if line[:2] == matrix_type_1:
                calib_matrix_1 = np.array(line[4:].strip().split(" ")).astype('float32').reshape(3,-1)
            elif line[:2] == matrix_type_2:
                calib_matrix_2 = np.array(line[4:].strip().split(" ")).astype('float32').reshape(3,-1)

    cam1 = calib_matrix_1[:,:3] # left image - P2
    cam2 = calib_matrix_2[:,:3] # right image - P3

    Tmat = np.array([0.54, 0., 0.])

    rev_proj_matrix = np.zeros((4,4))

    cv2.stereoRectify(cameraMatrix1 = cam1,cameraMatrix2 = cam2, \
                      distCoeffs1 = 0, distCoeffs2 = 0, \
                      imageSize = img_left_color.shape[:2], \
                      R = np.identity(3), T = Tmat, \
                      R1 = None, R2 = None, \
                      P1 =  None, P2 =  None, Q = rev_proj_matrix);


    points = cv2.reprojectImageTo3D(img, rev_proj_matrix)

    #reflect on x axis
    reflect_matrix = np.identity(3)
    reflect_matrix[0] *= -1
    points = np.matmul(points,reflect_matrix)

    #extract colors from image
    colors = cv2.cvtColor(img_left_color, cv2.COLOR_BGR2RGB)

    #filter by min disparity
    mask = img > img.min()
    out_points = points[mask]
    out_colors = colors[mask]

    #filter by dimension
    idx = np.fabs(out_points[:,0]) < 4.5
    out_points = out_points[idx]
    out_colors = out_colors.reshape(-1, 3)
    out_colors = out_colors[idx]

    return out_points, out_colors


class occupancy_marker(Node):
    def __init__(self):
        super().__init__('occupancy_publisher')

        self.color_cam1 = self.create_subscription(Image, '/my_camera_1/image_raw', self.imagecb_1, 10)
        self.color_cam2 = self.create_subscription(Image, '/my_camera_2/image_raw', self.imagecb_2, 10)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.cv_image_2 = None                                                            # colour raw image variable (from colorimagecb())
        self.cv_image_1 = None                                                            # colour raw image variable (from colorimagecb())
        self.bw_image_2 = None                                                            # colour raw image variable (from colorimagecb())
        self.bw_image_1 = None                                                            # colour raw image variable (from colorimagecb())
        image_processing_rate = 0.2
        self.timer = self.create_timer(image_processing_rate, self.process_occupancy)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_array = MarkerArray()

        self.count = 0
        self.markers_max = 2500

        self.points = self.generate_points()


    def imagecb_1(self, data):
        self.cv_image_1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bw_image_1 = cv2.cvtColor(self.cv_image_1, cv2.COLOR_BGR2GRAY)
    def imagecb_2(self, data):
        self.cv_image_2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bw_image_2 = cv2.cvtColor(self.cv_image_2, cv2.COLOR_BGR2GRAY)
    def generate_points(self):
        # Generate some random points for demonstration
        num_points = 100
        points = np.random.rand(num_points, 3) * 10  # 100 points in a 10x10x10 space
        return points

    def process_occupancy(self):
        shape = plain(self.cv_image_1,1)
        shape = plain(self.cv_image_2,2)

        self.out_points, self.out_colors = points_reconstruction(self.cv_image_1, self.bw_image_1, self.bw_image_2)
        marker_id = 0
        print(len(self.out_points))

        for x in range(self.out_points.shape[0]):
                tvec = self.out_points[x]
                xa = tvec[0]
                ya = -tvec[2]
                za = tvec[1]

                if math.sqrt(xa**2 + ya**2 + za**2) > 0.5:
                    continue

                marker = Marker()
                marker.header.frame_id = "camera_link_1"
                marker.header.time_stamp = self.get_clock().now().to_msg()
                marker.type = Marker.SPHERE_LIST
                marker.action = Marker.ADD
                marker.scale.x = 0.001
                marker.scale.y = 0.001
                marker.scale.z = 0.001
                marker.color.a = 1.0

                r,g,b = self.out_colors[x]

                marker.color.r = r/255
                marker.color.g = g/255
                marker.color.b = b/255


                marker.pose.orientation.w = 1.0
                marker.pose.position.x = xa
                marker.pose.position.y = ya
                marker.pose.position.z = za
                marker.id = marker_id
                marker_id += 1

                marker.points = list()

                for point in self.points:
                    p = Point()
                    p.x, p.y, p.z = point
                    marker.points.append(p)

                #if self.count > self.markers_max:
                #    self.marker_array.markers.pop(0)


                self.marker_array.markers.append(marker)
                self.count += 1

        # Renumber the marker IDs
        for i, m in enumerate(self.marker_array.markers):
            m.id = i

        self.publisher.publish(self.marker_array)

        print("procesing")
        #print(self.out_points, self.out_colors)





def main():
    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('occupancy_process')
    node.get_logger().info('Node created: Occupancy process')        # logging information

    occupancy_class = occupancy_marker()
    rclpy.spin(occupancy_class)

    occupancy_class.shutdown()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    main()
