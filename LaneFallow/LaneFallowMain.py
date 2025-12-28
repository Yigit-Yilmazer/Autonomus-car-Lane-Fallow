import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
from PinsandConnections import(
    set_servo_angle, move_robot,stop_motors,cleanup_PinsandConnections,initialize_camera,servo_pwm,pwm_a, pwm_b
)
def thresholding(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#adapting the light changes with HSV command
    low_white = np.array([0, 0, 150]) #low limit
    highest_white = np.array([179, 70, 255])#highest limit
    mask_white = cv2.inRange(imgHsv, low_white, highest_white)#give only two white lane 
    return mask_white


def warpImg(img , points , w , h):
    pts1 = np.float32(points)#4 corner on image
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])#bird eye view
    matrix = cv2.getPerspectiveTransform(pts1, pts2)#transformation matrix calculation
    img_warp = cv2.warpPerspective(img, matrix, (w, h))
    return img_warp


def get_histogram(img, min_per=0.1, region=2): 
    hist_values = np.sum(img[img.shape[0] // region:, :], axis=0)#I used Slicing syntax and give the colse area
    max_value = np.max(hist_values)#give max pixel sum
    min_value = min_per * max_value#give min pixel intensity and filter the noisy
    index_array = np.where(hist_values >= min_value)
    if len(index_array[0]) == 0:#if there is no lane assume that center min value > allPlace
        return img.shape[1] // 2
    base_point = int(np.average(index_array))
    return base_point




KP = 0.08  #vibration and stability setting
KD = 0.04  # reaction and clear turns



last_error = 0 
def get_lane_curve_and_steer(img):
    global last_error
    h, w, c = img.shape
    points = np.float32([(170, 300), (w - 170, 300), (100, h), (w - 100, h)])#I set according to cam angle
    img_thres = thresholding(img)#isolate the lane
    img_warp = warpImg(img_thres, points, w, h)#bird eye view
    base_point = get_histogram(img_warp) #lane x centerlocation
    img_center = w // 2 #center location
    



    #PD control
    error = base_point - img_center
    derivative = error - last_error
    steering_correction = (KP * error) + (KD * derivative)  #servo correction according KP and KD
    steering_angle = 90 - steering_correction #90deegre value is straight value
    last_error = error #give error 
    set_servo_angle(steering_angle)
    move_robot(35)

   
    img_warp_color = cv2.cvtColor(img_warp, cv2.COLOR_GRAY2BGR)#screen print 
    cv2.line(img_warp_color, (img_center, 0), (img_center, h), (255, 0, 0), 2)#verticle line see the axis
    cv2.circle(img_warp_color, (base_point, h - 25), 15, (0, 255, 0), cv2.FILLED)#circle which is represent robot on the screen
    
    return img_warp_color




if __name__ == '__main__':
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": (640, 480)},
        sensor={'output_size': (2328, 1748), 'bit_depth': 10}
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(1)
    
    set_servo_angle(90)
    time.sleep(1)   
    
    try:
        while True:
            #live view
            img_rgb = picam2.capture_array()
            img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            
            visualization_img = get_lane_curve_and_steer(img)
            
            cv2.imshow('Lane Visualization', visualization_img)
            cv2.imshow('Original Camera View', img)
            
            if cv2.waitKey(1) & 0xFF == ord('a'):
                break
    
    except KeyboardInterrupt:
        print("Program end")
    
    finally:
        print("The motors are stopped")
        stop_motors()
        servo_pwm.stop()
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        picam2.stop()
        cv2.destroyAllWindows()