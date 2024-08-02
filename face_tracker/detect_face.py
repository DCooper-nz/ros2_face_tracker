
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError


class DetectFace(Node):

    def __init__(self):
        super().__init__('detect_face')

        self.get_logger().info('Looking for the face...')
        self.image_sub = self.create_subscription(Image,"/image_in",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.face_pub  = self.create_publisher(Point,"/detected_face",1)
        self.get_logger().info("loading cascade")
        self.face_cascade = cv2.CascadeClassifier('/home/dave/ws_opencv/src/ros2_face_tracker/face_tracker/facedetection.xml') 
        self.bridge = CvBridge()


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv2.imshow("frame", cv_image)
        except CvBridgeError as e:
            print(e)

        try:
            
            frame = cv_image
            reading =self.face_cascade.detectMultiScale(frame)
            for (x,y,w,h) in reading:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            print(frame[0]) #print the co-ordinates to the terminal.
            # if frame.any():
            #     self.velocity_message.linear.x = 0.2
            #     self.pub_cmd.publish(self.velocity_message)
            # else:
            #     self.velocity_message.linear.x = 0.0
            #     self.pub_cmd.publish(self.velocity_message)


            cv2.imshow("frame",cv_image)

            

            img_to_pub = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            point_out = Point()

            # # Keep the biggest point
            # # They are already converted to normalised coordinates
            # for i, kp in enumerate(keypoints_norm):
            #     x = kp.pt[0]
            #     y = kp.pt[1]
            #     s = kp.size

            #     self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

            #     if (s > point_out.z):                    
            #         point_out.x = x
            #         point_out.y = y
            #         point_out.z = s

            # if (point_out.z > 0):
            #     self.face_pub.publish(point_out) 
        except CvBridgeError as e:
            print(e)  

def wait_on_gui():
    cv2.waitKey(2)

def main(args=None):

    rclpy.init(args=args)

    detect_face = DetectFace()
    while rclpy.ok():
        rclpy.spin_once(detect_face)
        wait_on_gui()

    detect_face.destroy_node()
    rclpy.shutdown()

