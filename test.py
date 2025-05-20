#!/usr/bin/env python
import rospy
import cv2
import os
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.sub = rospy.Subscriber("/hikrobot_camera/rgb", Image, self.callback)
        
        # 初始化保存参数
        self.save_count = 0
        self.save_dir = "./image/cali"
        os.makedirs(self.save_dir, exist_ok=True)  # 确保目录存在
        
        # 可选：从现有文件恢复编号
        self._init_save_counter()

    def _init_save_counter(self):
        """初始化时读取目录中已有文件的最大编号"""
        if os.path.exists(self.save_dir):
            existing_files = [f for f in os.listdir(self.save_dir) if f.startswith("captured_")]
            if existing_files:
                last_num = max([int(f.split('_')[1].split('.')[0]) for f in existing_files])
                self.save_count = last_num + 1

    def callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo_once("成功接收图像！")
        except Exception as e:
            rospy.logerr("转换失败: %s", str(e))

    def save_image(self):
        if self.image is not None:
            # 生成带序号和时间的文件名
            filename = os.path.join(
                self.save_dir,
                f"captured_{self.save_count}_{datetime.now().strftime('%H%M%S')}.jpg"
            )
            cv2.imwrite(filename, self.image)
            self.save_count += 1
            rospy.loginfo("已保存: %s", filename)
        else:
            rospy.logwarn("无图像数据可保存！")

if __name__ == "__main__":
    rospy.init_node("image_subscriber")
    sub = ImageSubscriber()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if sub.image is not None:
            Image_clone = sub.image.copy()
            # 缩小图像以适应窗口
            Image_clone = cv2.resize(Image_clone, (2560, 1440))
            cv2.imshow("Image", Image_clone)
            key = cv2.waitKey(1)
            if key == ord(' '):
                sub.save_image()
            elif key == 27:
                break
        rate.sleep()

    cv2.destroyAllWindows()