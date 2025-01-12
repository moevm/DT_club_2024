import cv2
def image_stylization(file_name, spatial_image=100, range_image=.1, out_name='out.jpg'):
    image = cv2.imread(file_name)
    stylized_image = cv2.stylization(image, sigma_s=spatial_image, sigma_r=range_image)
    cv2.imwrite(out_name, stylized_image)
if __name__ == '__main__':
    image_stylization('anime.jpg')  