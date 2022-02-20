#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time

size_of_pound = rospy.get_param('size_of_pound' , 48)
size_of_25piaster = rospy.get_param('size_of_25piaster' , 46)

arr = None
counter = 0
same_coin_threshold = 80

x_scale = rospy.get_param('x_scale' , 0.024609375)
y_scale = rospy.get_param('y_scale' , 0.02597222222)

camera_type = rospy.get_param('camera_type' , 0) # 2:for laptop cam , 0: for external cam


topic_to_publish = rospy.get_param('mrcamera' , 'mrcamera')
columnIndex = 1

home_array = np.zeros((1, 3))
home_array = np.uint16(np.around(home_array))


def y_correction(y):
    y_new = y - (((y-45)/141)*12)
    return y_new

def scaling(coins):
    """
    Scaling center of coins from pixels to real-life

    """
    coins_array = coins
    coins_array[:, 0] = ((coins_array[:, 0] * x_scale)*10)
    coins_array[:, 1] = ((coins_array[:, 1] * y_scale)*10)
    
    for i in range(coins.shape[0]):
        if coins_array[i][1] >= 45:
            coins_array[i][1]= y_correction(coins_array[i][1])
        if coins_array[i][2] >= size_of_pound:
            coins_array[i][2] = 0
        elif coins_array[i][2] <= size_of_25piaster:
            coins_array[i][2] = 1
        time.sleep(0.02)

    final_array = np.delete(coins_array, 3, 1)
    final_array = np.vstack((final_array,home_array))
    return final_array


def fill_array(detected_coins):
    """
    stack the coins of 5 frames and get best out of 5 predicition of the coin

    """
    global arr, counter
    if arr is None or counter == 0:
        arr = detected_coins
        arr = np.hstack((arr, np.ones((arr.shape[0], 1))))
        counter += 1
    else:
        counter += 1
        if counter > 10:
            arr = detected_coins
            arr = np.hstack((arr, np.ones((arr.shape[0], 1))))
        for coin in detected_coins:
            dists = arr[:, :2] - coin[:2].reshape(1, 2)
            dists = np.sum(dists * dists, axis=1)
            min_dist, min_idx = np.min(dists), np.argmin(dists)
            if min_dist < same_coin_threshold:
                arr[min_idx][2] += coin[2]
                arr[min_idx][3] += 1
            else:
                new_row = np.array([*coin, 1])
                arr = np.vstack((arr, new_row))
    to_return = np.copy(arr)
    to_return[:, 2] /= arr[:, 3]
    return np.round(to_return)


def camera(img):
    """
    image processing function
    """
    if img is None:
        print("Error opening image")
        return -1

    img_orig = img.copy()
    #cv.imshow("original", img)
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    img = cv.GaussianBlur(img, (21, 21), cv.BORDER_DEFAULT)

    all_circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    if all_circles is not None:
        all_circles_rounded = np.uint16(np.around(all_circles))
        re_shaped_coins = all_circles_rounded.reshape(all_circles_rounded.shape[1], 3)

        print("--------------------------------------------------------")
        print("i have found " + str(all_circles_rounded.shape[1]) + " coins")

        all_circles_rounded = fill_array(re_shaped_coins)
        all_circles_rounded = np.array(all_circles_rounded, dtype=np.uint16)
        detected_circles = all_circles_rounded[all_circles_rounded[:, 3] >= 2, :3]
        sorted_coins = detected_circles[detected_circles[:,columnIndex].argsort()]
        #print("before scaling:")
        #print(sorted_coins)

        coins_after = scaling(all_circles_rounded)
        #print("--------------------------------------------------------")

        print("after scaling:")
        print(coins_after)

        bridge = CvBridge()
        pub.publish(bridge.cv2_to_imgmsg(coins_after))

        for i in detected_circles:
            if(((i[2] >= size_of_pound and i[2] <= (size_of_pound + 4))) or (i[2] <= size_of_25piaster and i[2] >= (size_of_25piaster - 3))):
                img1 = cv.circle(img_orig, (i[0], i[1]), i[2], (50, 200, 300), 5)
                img2 = cv.circle(img_orig, (i[0], i[1]), 2, (255, 0, 0), 3)
            if (i[2] >= size_of_pound and i[2] <= (size_of_pound + 4)):
                cv.putText(img_orig, "Pound ", (i[0] - 70, i[1] + 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0),
                           2)  # with counter
                # cv.putText(img_orig , "Pound " + str(count_pounds) , (i[0]-70 , i[1]+30) , cv.FONT_HERSHEY_SIMPLEX , 0.5 , (255,255,255),2) #without counter
                # count_pounds += 1

            # if(i[2]< size_of_pound and i[2] > (size_of_25piaster)):
            #     cv.putText(img_orig , "50 piaster ", (i[0]-70 , i[1]+30) , cv.FONT_HERSHEY_SIMPLEX , 0.5 , (255,255,255),2) #with counter
            # cv.putText(img_orig , "50 piaster " + str(count_piaster) , (i[0]-70 , i[1]+30) , cv.FONT_HERSHEY_SIMPLEX , 0.5 , (255,255,255),2) #without counter
            # count_piaster += 1
            if (i[2] <= size_of_25piaster and i[2] >= (size_of_25piaster - 3)):
                cv.putText(img_orig, "25 piaster ", (i[0] - 70, i[1] + 30), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (0, 0, 255), 2)  # with counter
                # cv.putText(img_orig , "50 piaster " + str(count_piaster) , (i[0]-70 , i[1]+30) , cv.FONT_HERSHEY_SIMPLEX , 0.5 , (255,255,255),2) #without counter
                # count_piaster += 1
    else:
        bridge = CvBridge()
        pub.publish(bridge.cv2_to_imgmsg(home_array))
    cv.imwrite('/home/ahmed/scara/src/mrcamera/image/coins_after.jpg', img_orig)
    cv.imshow("text", img_orig)
    if cv.waitKey(1) == 27:  # esc Key
        return -1


def perception():
    """
    main function
    """

    global pub

    rospy.init_node('mrcamera', anonymous=True)
    pub = rospy.Publisher(topic_to_publish, Image, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    frameWidth = rospy.get_param('frameWidth' , 1280) 
    frameHeight = rospy.get_param('frameHeight' , 720) 
    cap = cv.VideoCapture(camera_type)  
    cap.set(cv.CAP_PROP_FRAME_WIDTH, frameWidth)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, frameHeight)
    
    #cap.set(frameWidth,3)
    #cap.set(frameHeight,4)

    while True:
        ret, img = cap.read()
        if camera(img) == -1:  # esc Key
            break

    cv.destroyAllWindows()
    rospy.spin()
    cap.release()
    cv.destroyAllWindows()
    rospy.spin()
    rate.sleep()


if __name__ == "__main__":
    try:
        perception()
    except rospy.ROSInterruptException:
        pass
