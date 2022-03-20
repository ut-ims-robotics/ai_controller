#!/usr/bin/env python
import rospy
import rospkg
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math

GUI_ENABLED = True

# Create the bridge that allows us to take Image type messages and convert them to OpenCV image format
bridge = cv_bridge.core.CvBridge()


# Create a boolean to keep track of whether we have a new image available or not
new_img_available = False

new_img_msg = Image()

def img_callback(image_msg):
    '''
    This is the callback function that gets called whenever
    a new Image type message arrives.
    '''
    global new_img_available, new_img_msg

    new_img_available = True
    new_img_msg = image_msg

def main():
    global new_img_available

    #Initialise a node with a unique name.
    rospy.init_node('ai_controller', anonymous=True)
    
    # Create a subscriber that starts listening for incoming messages
    # of the type Image on the usb_cam/image_raw topic
    rospy.Subscriber('image_raw', Image, img_callback)

    # Create a publisher for velocity commands
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Create a window where we will show the camera feed
    cv2.namedWindow('ai_debug_win')

    rospack = rospkg.RosPack()
    pkgpath = rospack.get_path('ai_controller')

    # Load Yolo neural network
    net = cv2.dnn.readNet(pkgpath + "/resources/yolov3-tiny.weights", pkgpath + "/resources/yolov3-tiny.cfg")
    classes = []
    with open(pkgpath + "/resources/coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # Spin as long as rospy still runs.
    while not rospy.is_shutdown():
        # Only process a fresh image when we have one received
        if new_img_available:
            # Convert the image from Image message to OpenCV image format
            cv_image = bridge.imgmsg_to_cv2(new_img_msg, desired_encoding='bgr8')

            height, width, channels = cv_image.shape
            # Detecting objects
            blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)

            # Showing informations on the screen
            class_ids = []
            confidences = []
            boxes = []
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            font = cv2.FONT_HERSHEY_PLAIN

            center_x = None
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    color = colors[i]
                    if (GUI_ENABLED):
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
                        cv2.putText(cv_image, label, (x, y + 30), font, 3, color, 3)

                    # Store the last detected person's center location on thd horizontal axis
                    if(label == "person"):
                        center_x = (x+w/2)

            if (center_x != None):
                #Driving logic: keep object at the center of the screen
                print("Object center_x: ", center_x, width)
                cmd_vel_msg = Twist()
                cmd_vel_msg.angular.z = (width/2 - center_x) * math.pi / width
                cmd_vel_pub.publish(cmd_vel_msg)


            # Set the boolean to False, indicating that we have used up the camera image
            new_img_available = False

            # Show the image in the window we created before the while-loop
            if (GUI_ENABLED):
                cv2.imshow('ai_debug_win', cv_image)

        # Show the image for 1 ms.
        if (GUI_ENABLED):
            if (cv2.waitKey(1) == 27):
                break # Exit if esc pressed

    # Close nicely
    if (GUI_ENABLED):
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
