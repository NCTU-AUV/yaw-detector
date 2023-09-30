import cv2
import numpy as np

img = cv2.imread("tile_0.png")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 250, 300, apertureSize=3)
lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

# calculate yaw angle
data = lines[:, 0, :]

np.set_printoptions(suppress=True)

theta_mean = np.mean(data[:, 1])

smaller_data = data[data[:, 1] < theta_mean]
greater_data = data[data[:, 1] >= theta_mean]

sorted_indices = np.argsort(smaller_data[:, 0])
smaller_data = smaller_data[sorted_indices]

sorted_indices = np.argsort(greater_data[:, 0])
greater_data = greater_data[sorted_indices]

def get_step(data):
    tot = 0
    for x in range(1, len(data)):
        tot += (data[x, 0] - data[x-1, 0])
    tot /= len(data)-1
    return tot

smaller_step = get_step(smaller_data)
greater_step = get_step(greater_data)

if smaller_step > greater_step:
    data = smaller_data
else:
    data = greater_data

yaw = np.rad2deg(np.mean(data[:, 1])) - 90
print(yaw)

# draw red lines
for r_theta in lines:
    arr = np.array(r_theta[0], dtype=np.float64)
    r, theta = arr

    a = np.cos(theta)
    b = np.sin(theta)

    x0 = a * r
    y0 = b * r

    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))

    cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

cv2.imwrite("linesDetected.jpg", img)
