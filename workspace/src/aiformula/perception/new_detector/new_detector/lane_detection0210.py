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
        
        # 发布器，匹配YOLOP版本的输出
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
        
        # 进行车道线检测（包含减少光照影响和俯瞰图变换）
        annotated_image, lane_mask = self.detect_lane_lines(cv_image)
        
        try:
            # 发布标注后的图像（类型bgr8）
            annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_image_msg.header.stamp = msg.header.stamp
            self.annotated_mask_image_pub.publish(annotated_image_msg)
            
            # 发布分割掩码（类型mono8）
            lane_mask_msg = self.cv_bridge.cv2_to_imgmsg(lane_mask, "mono8")
            lane_mask_msg.header.stamp = msg.header.stamp
            self.lane_mask_image_pub.publish(lane_mask_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image to ROS message: {e}")

    def detect_lane_lines(self, image):
        # ------------------------------
        # 1. 减少光照影响：CLAHE 处理
        # ------------------------------
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        norm_image = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

        # ------------------------------
        # 2. 俯瞰图（鸟瞰图）变换
        # ------------------------------
        height, width = norm_image.shape[:2]
        # 根据 ZED X 相机拍摄图像，进一步扩大梯形采样范围
        src_points = np.float32([
            [width * 0.1, height * 0.2],   # 左上角
            [width * 0.9, height * 0.2],   # 右上角
            [width * 0.9, height],         # 右下角
            [width * 0.1, height]          # 左下角
        ])
        # 目标点保持为一个矩形区域
        dst_points = np.float32([
            [width * 0.2, 0],
            [width * 0.8, 0],
            [width * 0.8, height],
            [width * 0.2, height]
        ])
        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_image = cv2.warpPerspective(norm_image, M, (width, height))

        # 使用变换后的图像作为后续处理的基础
        annotated_image = warped_image.copy()

        # ------------------------------
        # 3. 车道线检测流程
        # ------------------------------
        # 转换为灰度图
        gray_image = cv2.cvtColor(warped_image, cv2.COLOR_BGR2GRAY)
        # 应用高斯模糊
        blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
        # Canny 边缘检测
        edges = cv2.Canny(blurred_image, 100, 200)
        
        # 定义感兴趣区域（ROI），仍然选取图像下半部分
        mask = np.zeros_like(edges)
        roi_polygon = np.array([[
            (100, height),
            (width-100, height),
            (width-100, int(height/2)),
            (100, int(height/2))
        ]], np.int32)
        cv2.fillPoly(mask, roi_polygon, 255)
        
        # 应用 ROI 掩码
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # 创建车道线分割掩码
        lane_mask = np.zeros_like(edges)
        
        # 使用霍夫变换检测直线
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 
                                threshold=50, minLineLength=50, maxLineGap=200)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 在标注图像上绘制车道线（绿色）
                cv2.line(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                # 在分割掩码上绘制车道线（白色，线宽为5）
                cv2.line(lane_mask, (x1, y1), (x2, y2), 255, 5)
        
        # 形态学处理使车道线更平滑
        kernel = np.ones((5, 5), np.uint8)
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
