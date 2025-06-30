import cv2

def rotate_and_resize_cv2_optimized(input_path, output_path):
    img = cv2.imread(input_path)
    if img is None:
        raise ValueError("无法读取图片")
    # 缩放
    resized_img = cv2.resize(img, (1500, 2800), interpolation=cv2.INTER_LANCZOS4)

    cv2.imwrite(output_path, resized_img)
rotate_and_resize_cv2_optimized('/home/tjurm/Code/TJURM-Radar/config/models/output.png', '/home/tjurm/Code/TJURM-Radar/config/models/output.png')