import rclpy
import cv2
import numpy as np
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
        self.face_capture_pub = self.create_publisher(Image, "/face_capture", 1)
        self.face_pub  = self.create_publisher(Point,"/detected_face",1)
        self.get_logger().info("loading model")
        self.net = cv2.dnn.readNetFromCaffe('/home/dave/ws_opencv/src/ros2_face_tracker/face_tracker/deploy.prototxt.txt', '/home/dave/ws_opencv/src/ros2_face_tracker/face_tracker/res10_300x300_ssd_iter_140000.caffemodel')
        #self.face_cascade = cv2.CascadeClassifier('/home/dave/ws_opencv/src/ros2_face_tracker/face_tracker/facedetection.xml') 
        self.bridge = CvBridge()


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv2.imshow("frame", cv_image)
        except CvBridgeError as e:
            print(e)

        try:
            
            
            image = cv_image
            face = cv_image
            faceZ = 0
            
            (h, w) = image.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 1.0,
                (300, 300), (104.0, 177.0, 123.0))
           
            
            print("[INFO] computing object detections...")
            self.net.setInput(blob)
            detections = self.net.forward()

            # loop over the detections
            for i in range(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated with the
                # prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the `confidence` is
                # greater than the minimum confidence
                if confidence > 0.5:
                    # compute the (x, y)-coordinates of the bounding box for the
                    # object
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                                       #center_coordinates = x + w // 2, y + h // 2
                    faceCentre = (startX + endX) //2, (startY + endY) //2
                    faceZ = 1800 - ((endX - startX) + (endY - startY) * 3.5)

                    face = image.copy()[startY:endY, startX:endX]
                    

                    # draw the bounding box of the face along with the associated
                    # probability
                    text = "{:.2f}%".format(confidence * 100)
                    y = startY - 10 if startY - 10 > 10 else startY + 10
                    cv2.rectangle(image, (startX, startY), (endX, endY),
                        (0, 0, 255), 2)  
                    
                    cv2.circle(image, (faceCentre), 5 ,(0, 255, 0), -2)
                    
                    cv2.putText(image, text, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                    cv2.putText(image, str(faceZ), (faceCentre),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                                        
                                           
                   
                    try:
                        face = cv2.resize(face, (512,512))
                    except:
                        face = cv_image
            # show the output image
            cv2.imshow("Output", image)




            img_to_pub = self.bridge.cv2_to_imgmsg(image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)
            face_to_pub = self.bridge.cv2_to_imgmsg(face, "bgr8")
            face_to_pub.header = data.header
            self.face_capture_pub.publish(face_to_pub)

            point_out = Point()
            if (faceZ > 0):
                point_out.x = float(faceCentre[0])
                point_out.y = float(faceCentre[1])
                point_out.z = faceZ

            if (point_out.z > 0):
               self.face_pub.publish(point_out) 
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
