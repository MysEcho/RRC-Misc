import cv2 
#import open3d as o3d
#import matplotlib.pyplot as plt

#cap =cv2.VideoCapture(0)
count =1
while True:
    #ret, frame =cap.read()
    #cv2.imshow('frame', frame)
    #cv2.imwrite("frames/frame%d.jpg" % count, frame)
    #image = cv2.imread("frames/frame%d.jpg" % count)
    image = cv2.imread("rgbd-scenes/meeting_small/meeting_small_1/meeting_small_1_29.png")
    #gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    cv2.imshow('Grayscale', image)
    
    count = count + 1
    key=cv2.waitKey(1)
    if key == 27:
        break
    '''if cv2.waitKey(1) & 0xFF == ord('q'):
        break'''
    '''color_raw = o3d.io.read_image(
        "apple_1/rgbd-dataset/apple/apple_1/apple_1_1_%d.png" % count)
    depth_raw = o3d.io.read_image(
        "apple_1/rgbd-dataset/apple/apple_1/apple_1_1_%d_depth.png" % count)
    rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(
        color_raw, depth_raw)
    print(rgbd_image)
    plt.subplot(1, 2, 1)
    plt.title('SUN grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('SUN depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()

    count = count +1
    if count == 196:
        break'''

cap.release()
cv2.destroyAllWindows()