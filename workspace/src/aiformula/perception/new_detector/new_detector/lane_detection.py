import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.cv_bridge = CvBridge()
        buffer_size = 10
        
        # 修改发布器，匹配YOLOP版本的输出
        self.annotated_mask_image_pub = self.create_publisher(
            Image, 'pub_annotated_mask_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(
            Image, 'pub_mask_image', buffer_size)
            
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        # 进行车道线检测和分割
        annotated_image, lane_mask = self.detect_lane_lines(cv_image)
        
        try:
            # 发布标注后的图像
            annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            self.annotated_mask_image_pub.publish(annotated_image_msg)
            
            # 发布分割掩码
            lane_mask_msg = self.cv_bridge.cv2_to_imgmsg(lane_mask, "mono8")
            lane_mask_msg.header.stamp = msg.header.stamp
            self.lane_mask_image_pub.publish(lane_mask_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image to ROS message: {e}")

    def detect_lane_lines(self, image):
        # 保存原始图像副本用于标注
        annotated_image = image.copy()
        
        # 转换为灰度图
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 应用高斯模糊
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        
        # Canny边缘检测
        edges = cv2.Canny(blurred_image, 30, 100)
        
        # 定义感兴趣区域(ROI)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[(100, height), 
                           (width-100, height), 
                           (width-100, int(height/2)), 
                           (100, int(height/2))]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        
        # 应用ROI掩码
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # 创建分割掩码
        lane_mask = np.zeros_like(edges)
        
        # 使用霍夫变换检测直线
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 
                               threshold=50, minLineLength=50, maxLineGap=200)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 在标注图像上绘制线条
                cv2.line(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                # 在分割掩码上绘制线条
                cv2.line(lane_mask, (x1, y1), (x2, y2), 255, 5)
        
        # 对分割掩码进行形态学操作，使其更平滑
        kernel = np.ones((5,5), np.uint8)
        lane_mask = cv2.dilate(lane_mask, kernel, iterations=1)
        lane_mask = cv2.erode(lane_mask, kernel, iterations=1)
        
        return annotated_image, lane_mask

def main():
    rclpy.init()
    lane_detection_node = LaneDetectionNode()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
