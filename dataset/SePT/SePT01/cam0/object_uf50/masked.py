# opencv 图像的基本运算 (mask)
 
# 导入库
import numpy as np
import argparse
import cv2
 
# 构建参数解析器
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
ap.add_argument("-n", "--name", required=True, help="Image name")
args = vars(ap.parse_args())
 
# 加载图像
image = cv2.imread(args["image"])
cv2.imshow("image loaded", image)
 
# 创建矩形区域，边框为黑色0
rectangle = np.zeros(image.shape[0:2], dtype="uint8")
# mask1 = cv2.rectangle(rectangle, (75, 435), (115, 475), 255, -1)
# mask2 = cv2.rectangle(rectangle, (290, 440), (330, 480), 255, -1)
# mask3 = cv2.rectangle(rectangle, (530, 460), (560, 490), 255, -1)
# mask4 = cv2.rectangle(rectangle, (685, 445), (735, 495), 255, -1)
# mask5 = cv2.rectangle(rectangle, (880, 440), (940, 500), 255, -1)
# mask6 = cv2.rectangle(rectangle, (930, 470), (1020, 550), 255, -1)
# mask7 = cv2.rectangle(rectangle, (180, 470), (270, 550), 255, -1)
# mask8 = cv2.rectangle(rectangle, (160, 490), (300, 600), 255, -1)
x, y, w, h = cv2.selectROI(image)
mask = cv2.rectangle(rectangle, (x, y), (x + w, y +h), 255, -1)

# 使用mask
# cv2.imshow("Mask", mask)
 
# Apply out mask -- notice how only the person in the image is cropped out
masked = cv2.bitwise_and(image, image, mask=mask)
cv2.imwrite(args["name"]+".png", masked)
cv2.imshow("Mask Applied to Image", masked)
cv2.waitKey(0)
cv2.destroyAllWindows()
