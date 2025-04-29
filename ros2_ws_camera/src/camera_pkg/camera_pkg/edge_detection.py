#!~/rl_envsss/bin python3
# with mask for debuging
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os
import cv2
import math
import numpy as np
import statistics
from std_msgs.msg import String
import json
import traceback # For better error logging

# --- Constants and Boundary Definitions (Keep as before or adjust) ---
line1 = [(291+10-50, 0), (151+40-40, 144)]
line2 = [(241+10-50, 0), (101+10-40, 144)]
line3 = [(338+40, 0), (430+65, 144)]
line4 = [(388+35, 0), (480+120, 144)]
line_left_coords  = [(241+10-50, 0), (101+10-40, 144)]
line_right_coords = [(388+35, 0), (480+120, 144)]
length_threshold = 50
angle_similarity_threshold = 25
# --- End Constants ---

# --- Helper Functions (Keep as before, including the new filter_mask_boundary_lines) ---
def get_roi_mask(img):
    height, width = img.shape[:2]
    left_line_p1 = (int(width * 0.25), 0)
    left_line_p2 = (0, height)
    right_line_p1 = (int(width * 0.75), 0)
    right_line_p2 = (width, height)
    mask = np.zeros_like(img[:, :, 0] if len(img.shape) == 3 else img, dtype=np.uint8)
    vertices = np.array([left_line_p1, left_line_p2, right_line_p2, right_line_p1], dtype=np.int32)
    vertices = vertices.reshape((-1, 1, 2))
    cv2.fillPoly(mask, [vertices], 255)
    return mask

def calculate_line_points(midpoint, angle_deg, width, height):
    mx, my = midpoint
    theta = np.deg2rad(angle_deg)
    intersections = []
    if abs(np.sin(theta)) > 1e-6:
        t = (0 - my) / np.sin(theta)
        x = mx + t * np.cos(theta)
        if 0 <= x < width: intersections.append((int(x), 0))
        t = (height - 1 - my) / np.sin(theta)
        x = mx + t * np.cos(theta)
        if 0 <= x < width: intersections.append((int(x), height - 1))
    if abs(np.cos(theta)) > 1e-6:
        t = (0 - mx) / np.cos(theta)
        y = my + t * np.sin(theta)
        if 0 <= y < height: intersections.append((0, int(y)))
        t = (width - 1 - mx) / np.cos(theta)
        y = my + t * np.sin(theta)
        if 0 <= y < height: intersections.append((width - 1, int(y)))
    unique_intersections = []
    for p in intersections:
        is_unique = True
        for up in unique_intersections:
            if math.hypot(p[0]-up[0], p[1]-up[1]) < 1.0: is_unique = False; break
        if is_unique: unique_intersections.append(p)
    if len(unique_intersections) >= 2: return unique_intersections[0], unique_intersections[1]
    return None, None

def draw_lane_lines(img, left_line=None, right_line=None, color=(0, 0, 255), thickness=2):
    image_shape = img.shape
    height, width = image_shape[:2]
    def draw_line(line_data):
        if line_data is None: return
        midpoint = line_data.get('midpoint')
        angle = line_data.get('angle')
        if midpoint is None or angle is None: return
        start, end = calculate_line_points(midpoint, angle, width, height)
        if start and end: cv2.line(img, start, end, color, thickness)
    draw_line(left_line)
    draw_line(right_line)
    return img

def calculate_line_angle_degrees(p1, p2):
    """ Calculates angle in degrees [0, 360). """
    x1, y1 = p1; x2, y2 = p2
    if x1 == x2 and y1 == y2: return 0.0
    angle_rad = math.atan2(y2 - y1, x2 - x1)
    angle_deg = math.degrees(angle_rad)
    return angle_deg % 360 # Normalize to [0, 360)

def angle_difference_degrees(angle1, angle2):
    diff = abs(angle1 - angle2)
    return min(diff, 360 - diff)

def get_line_midpoint(line):
    x1 = line.get('x1', 0.0); y1 = line.get('y1', 0.0)
    x2 = line.get('x2', 0.0); y2 = line.get('y2', 0.0)
    mid_x = (x1 + x2) / 2.0; mid_y = (y1 + y2) / 2.0
    return mid_x, mid_y

def average_angle_vectorized(angles_deg):
    if not angles_deg: return None
    sum_cos = 0.0; sum_sin = 0.0
    valid_angles = [a for a in angles_deg if a is not None]
    if not valid_angles: return None
    for angle in valid_angles:
        rad = math.radians(angle)
        sum_cos += math.cos(rad); sum_sin += math.sin(rad)
    count = len(valid_angles)
    avg_cos = sum_cos / count; avg_sin = sum_sin / count
    if abs(avg_sin) < 1e-9 and abs(avg_cos) < 1e-9: return 0.0
    avg_rad = math.atan2(avg_sin, avg_cos)
    avg_deg = math.degrees(avg_rad)
    return avg_deg % 360

def filter_lines_by_length(lines, min_length):
    return [line for line in lines if line.get('length', 0) >= min_length]

def point_to_line_distance(px, py, x1, y1, x2, y2):
    line_mag_sq = (x2 - x1)**2 + (y2 - y1)**2
    if line_mag_sq < 1e-9: return math.hypot(px - x1, py - y1)
    A = y2 - y1; B = x1 - x2; C = -A * x1 - B * y1
    denom_sq = A**2 + B**2
    if denom_sq < 1e-9: return math.hypot(px - x1, py - y1)
    distance = abs(A * px + B * py + C) / math.sqrt(denom_sq)
    return distance

def get_line_properties(segments):
    """ Calculates properties for each detected line segment. """
    line_properties = []
    for idx, segment in enumerate(segments):
        if segment is None or len(segment) == 0 or len(segment[0]) != 4: continue
        x1, y1, x2, y2 = segment[0]
        try: x1f, y1f, x2f, y2f = map(float, [x1, y1, x2, y2])
        except ValueError: continue
        if abs(x1f - x2f) < 1e-6 and abs(y1f - y2f) < 1e-6:
             length = 0.0; angle = 0.0; angle_normalized_180 = 0.0
        else:
            length = math.hypot(x2f - x1f, y2f - y1f)
            angle = calculate_line_angle_degrees((x1f, y1f), (x2f, y2f))
            angle_normalized_180 = math.degrees(math.atan2(y2f - y1f, x2f - x1f))
        line_properties.append({
            "line_number": idx + 1, "x1": round(x1f), "y1": round(y1f), "x2": round(x2f), "y2": round(y2f),
            "length": length, "angle": angle, "angle_normalized": angle_normalized_180
        })
    return line_properties

def get_roi(image, roi_y_start=0.5):
    if image is None: return None
    try: height, width = image.shape[:2]
    except AttributeError: return None
    roi_y = int(height * roi_y_start)
    if roi_y >= height: roi_y = height - 1
    if roi_y < 0 : roi_y = 0
    roi = image[roi_y:, :]
    return roi

def draw_the_lines_from_line_prop(img, line_properties, color=(0, 255, 0), thickness=2, with_number = False):
    if line_properties:
        for line in line_properties:
            if all(k in line for k in ('x1', 'y1', 'x2', 'y2')):
                try:
                    p1 = (int(line['x1']), int(line['y1'])); p2 = (int(line['x2']), int(line['y2']))
                    cv2.line(img, p1, p2, color, thickness)
                    if with_number and 'line_number' in line:
                        mid_x = int((p1[0] + p2[0]) / 2); mid_y = int((p1[1] + p2[1]) / 2)
                        cv2.putText(img, str(line['line_number']), (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                except (TypeError, ValueError) as e: print(f"Warning: Error drawing line {line.get('line_number', '?')}: {e}")
    return img

def filter_mask_boundary_lines(line_properties, roi_height, roi_width, angle_tolerance_deg=10.0, distance_tolerance_px=10.0):
    if roi_height <= 0 or roi_width <= 0: return line_properties
    mask_left_p1 = (int(roi_width * 0.25), 0); mask_left_p2 = (0, roi_height)
    mask_right_p1 = (int(roi_width * 0.75), 0); mask_right_p2 = (roi_width, roi_height)
    try:
        mask_left_angle = calculate_line_angle_degrees(mask_left_p1, mask_left_p2)
        mask_right_angle = calculate_line_angle_degrees(mask_right_p1, mask_right_p2)
    except Exception as e: print(f"Warning: Could not calculate mask boundary angles: {e}"); return line_properties
    filtered_lines = []
    for line in line_properties:
        line_mid_x, line_mid_y = get_line_midpoint(line)
        line_angle = line.get('angle')
        if line_angle is None: continue
        angle_diff_left = angle_difference_degrees(line_angle, mask_left_angle)
        dist_to_left = point_to_line_distance(line_mid_x, line_mid_y, mask_left_p1[0], mask_left_p1[1], mask_left_p2[0], mask_left_p2[1])
        if angle_diff_left < angle_tolerance_deg and dist_to_left < distance_tolerance_px: continue
        angle_diff_right = angle_difference_degrees(line_angle, mask_right_angle)
        dist_to_right = point_to_line_distance(line_mid_x, line_mid_y, mask_right_p1[0], mask_right_p1[1], mask_right_p2[0], mask_right_p2[1])
        if angle_diff_right < angle_tolerance_deg and dist_to_right < distance_tolerance_px: continue
        filtered_lines.append(line)
    return filtered_lines
# --- End Helper Functions ---


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.last_save_time = time.time()
        self.save_interval = 0.1
        self.save_dir = "/mnt/d/turtlebot_images"
        self.detected_lane_data_publisher = self.create_publisher(String, 'detected_lane_data', 10)
        try:
            if not os.path.exists(self.save_dir): os.makedirs(self.save_dir); self.get_logger().info(f"Created save directory: {self.save_dir}")
        except OSError as e: self.get_logger().error(f"Could not create save directory {self.save_dir}: {e}"); self.save_dir = None
        self.get_logger().info("Camera Subscriber Node Initialized.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e: self.get_logger().error(f"Error converting ROS image: {e}", throttle_duration_sec=5.0); return
        if cv_image is None: self.get_logger().warn("Converted cv_image is None.", throttle_duration_sec=5.0); return

        try:
            # --- Image Processing ---
            original_image_full = cv_image.copy()
            processed_image = cv_image.copy()

            # Apply ROI
            roi_image = get_roi(processed_image, 0.7)
            if roi_image is None or roi_image.size == 0:
                 self.get_logger().warn("ROI resulted in empty image.", throttle_duration_sec=5.0)
                 self.publish_detected_lane_data([])
                 return

            # *** START COLOR MASKING ***
            try:
                hsv_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            except cv2.error as e:
                 self.get_logger().error(f"Error converting ROI to HSV: {e}. ROI shape: {roi_image.shape}")
                 self.publish_detected_lane_data([])
                 return

            # *** ADJUSTED HSV RANGE FOR WHITE ***
            # Lower the minimum Value to catch less bright white parts
            lower_white = np.array([0, 0, 150])   # Lowered V from 190 to 180
            # Slightly increase max Saturation for less pure white
            upper_white = np.array([180, 80, 255]) # Increased S from 40 to 50
            # ************************************

            white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

            # *** Optional: Morphological Closing to fill gaps ***
            # kernel_size = 5 # Adjust kernel size as needed (odd numbers usually)
            # kernel = np.ones((kernel_size, kernel_size), np.uint8)
            # white_mask_closed = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
            # Use white_mask_closed below instead of white_mask if enabling this
            # ***************************************************

            # *** END COLOR MASKING ***

            corner_mask = get_roi_mask(roi_image)
            # Use the potentially closed mask here if enabled above
            final_mask = cv2.bitwise_and(white_mask, white_mask, mask=corner_mask)
            # final_mask = cv2.bitwise_and(white_mask_closed, white_mask_closed, mask=corner_mask) # If using closing

            # --- Detect lines on the FINAL MASK ---
            lsd = cv2.createLineSegmentDetector()
            try:
                lines_data = lsd.detect(final_mask)
            except cv2.error as e:
                self.get_logger().error(f"Error during LSD detect on mask: {e}")
                lines_data = None

            # --- Line Processing and Filtering ---
            line_properties = []
            final_left_line = None
            final_right_line = None

            if lines_data is not None and lines_data[0] is not None and len(lines_data[0]) > 0:
                lines = lines_data[0]
                line_properties = get_line_properties(lines)
                line_properties = filter_lines_by_length(line_properties, 50)
                roi_h, roi_w = final_mask.shape[:2]
                line_properties = filter_mask_boundary_lines(line_properties, roi_h, roi_w)

                # --- Calculate Representative Left/Right Lines ---
                if line_properties:
                    image_height, image_width = final_mask.shape[:2]
                    image_center_x = image_width / 2.0
                    try:
                        angle_left_ref = calculate_line_angle_degrees(line_left_coords[0], line_left_coords[1])
                        angle_right_ref = calculate_line_angle_degrees(line_right_coords[0], line_right_coords[1])
                    except Exception as e: angle_left_ref, angle_right_ref = 135.0, 45.0

                    left_lines, right_lines = [], []
                    for line in line_properties:
                        try: avg_x = (line.get('x1',0) + line.get('x2',0)) / 2.0
                        except TypeError: continue
                        if avg_x < image_center_x: left_lines.append(line)
                        else: right_lines.append(line)

                    # Filter and average Left Lines
                    filtered_left_lines = []
                    for line in left_lines:
                        if line.get('length', 0) <= length_threshold: continue
                        line_angle = line.get('angle');
                        if line_angle is None: continue
                        diff = angle_difference_degrees(line_angle, angle_left_ref)
                        if diff > angle_similarity_threshold: continue
                        filtered_left_lines.append(line)
                    if filtered_left_lines:
                        sorted_left = sorted(filtered_left_lines, key=lambda l: l.get('length', 0), reverse=True)
                        top_left = sorted_left[:2]
                        top_left_angles = [l.get('angle') for l in top_left]
                        avg_left_angle = average_angle_vectorized(top_left_angles)
                        if avg_left_angle is not None:
                            try:
                                avg_left_mid_x = statistics.mean([get_line_midpoint(l)[0] for l in top_left])
                                avg_left_mid_y = statistics.mean([get_line_midpoint(l)[1] for l in top_left])
                                distance_from_center = image_center_x - avg_left_mid_x
                                final_left_line = {'side': 'left', 'angle': avg_left_angle, 'midpoint': (avg_left_mid_x, avg_left_mid_y), 'num_lines_averaged': len(top_left), 'distance_from_center': distance_from_center}
                            except Exception as e: self.get_logger().warn(f"Error calculating left line stats: {e}")

                    # Filter and average Right Lines
                    filtered_right_lines = []
                    for line in right_lines:
                        if line.get('length', 0) <= length_threshold: continue
                        line_angle = line.get('angle');
                        if line_angle is None: continue
                        diff = angle_difference_degrees(line_angle, angle_right_ref)
                        if diff > angle_similarity_threshold: continue
                        filtered_right_lines.append(line)
                    if filtered_right_lines:
                        sorted_right = sorted(filtered_right_lines, key=lambda l: l.get('length', 0), reverse=True)
                        top_right = sorted_right[:2]
                        top_right_angles = [l.get('angle') for l in top_right]
                        avg_right_angle = average_angle_vectorized(top_right_angles)
                        if avg_right_angle is not None:
                            try:
                                avg_right_mid_x = statistics.mean([get_line_midpoint(l)[0] for l in top_right])
                                avg_right_mid_y = statistics.mean([get_line_midpoint(l)[1] for l in top_right])
                                distance_from_center = avg_right_mid_x - image_center_x
                                final_right_line = {'side': 'right', 'angle': avg_right_angle, 'midpoint': (avg_right_mid_x, avg_right_mid_y), 'num_lines_averaged': len(top_right), 'distance_from_center': distance_from_center}
                            except Exception as e: self.get_logger().warn(f"Error calculating right line stats: {e}")

            # --- Prepare Data for Publishing ---
            final_lines_list = []
            if final_left_line: final_lines_list.append(final_left_line)
            if final_right_line: final_lines_list.append(final_right_line)
            self.publish_detected_lane_data(final_lines_list)

            # --- Visualization ---
            vis_image = roi_image.copy()
            vis_image = draw_lane_lines(vis_image, final_left_line, final_right_line, color=(255, 0, 0), thickness=3)

            # Display the visualization stages
            cv2.imshow('Debug: White Mask', white_mask) # Show original white mask
            # Optionally show the closed mask if you enable it
            # cv2.imshow('Debug: Closed White Mask', white_mask_closed)
            cv2.imshow('Debug: Final Mask (White + Corner)', final_mask)
            cv2.imshow('Lane Detection Result', vis_image)
            cv2.waitKey(1)

        except CvBridgeError as e: self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image callback: {e}")
            self.get_logger().error(traceback.format_exc())
            try: self.publish_detected_lane_data([])
            except Exception as pub_e: self.get_logger().error(f"Also failed to publish empty list on error: {pub_e}")


    def publish_detected_lane_data(self, data):
        """Publish data immediately when called"""
        try:
            publishable_data = []
            for item in data:
                new_item = item.copy()
                if 'midpoint' in new_item and isinstance(new_item['midpoint'], tuple):
                    new_item['midpoint'] = list(new_item['midpoint'])
                publishable_data.append(new_item)
            json_str = json.dumps(publishable_data)
            msg = String(); msg.data = json_str
            self.detected_lane_data_publisher.publish(msg)
        except TypeError as e: self.get_logger().error(f"Failed to serialize lane data to JSON: {e}. Data: {data}")
        except Exception as e: self.get_logger().error(f"Error publishing lane data: {e}")


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    try: rclpy.spin(camera_subscriber)
    except KeyboardInterrupt: print("Keyboard interrupt received, shutting down.")
    finally:
        print("Closing OpenCV windows...")
        cv2.destroyAllWindows()
        print("Destroying node...")
        camera_subscriber.destroy_node()
        if rclpy.ok(): print("Shutting down ROS 2..."); rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
