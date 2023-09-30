import cv2
import numpy as np


def calculate_yaw_angle(image_path):
    image = cv2.imread(image_path)
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grayscale, 250, 300, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

    data = lines[:, 0, :]
    mean_theta = np.mean(data[:, 1])

    smaller_data = data[data[:, 1] < mean_theta]
    greater_data = data[data[:, 1] >= mean_theta]

    smaller_data = smaller_data[np.argsort(smaller_data[:, 0])]
    greater_data = greater_data[np.argsort(greater_data[:, 0])]

    smaller_step = np.mean(np.diff(smaller_data[:, 0]))
    greater_step = np.mean(np.diff(greater_data[:, 0]))

    selected_data = smaller_data if smaller_step > greater_step else greater_data

    return np.rad2deg(np.mean(selected_data[:, 1])) - 90


def draw_red_lines(image_path, output_path):
    image = cv2.imread(image_path)
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grayscale, 250, 300, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

    for r_theta in lines:
        r, theta = r_theta[0]
        a, b = np.cos(theta), np.sin(theta)
        x0, y0 = a * r, b * r
        x1, y1 = int(x0 + 1000 * (-b)), int(y0 + 1000 * (a))
        x2, y2 = int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))
        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    cv2.imwrite(output_path, image)


if __name__ == "__main__":
    print("90:", calculate_yaw_angle("tile_pos90.png"))
    print("75:", calculate_yaw_angle("tile_pos75.png"))
    print("45:", calculate_yaw_angle("tile_pos45.png"))
    print("30:", calculate_yaw_angle("tile_pos30.png"))
    print("15:", calculate_yaw_angle("tile_pos15.png"))
    print("0:", calculate_yaw_angle("tile_0.png"))
    print("-15:", calculate_yaw_angle("tile_neg15.png"))
    print("-30:", calculate_yaw_angle("tile_neg30.png"))
    print("-45:", calculate_yaw_angle("tile_neg45.png"))
    print("-75:", calculate_yaw_angle("tile_neg75.png"))
    print("-90:", calculate_yaw_angle("tile_neg90.png"))

    # draw_red_lines("tile_neg30.png", "linesDetected.jpg")
