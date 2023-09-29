import cv2
import numpy as np

img = cv2.imread("tile45.png")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 250, 300, apertureSize=3)
lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

# calculate yaw angle
thetas = lines[:, 0, 1]
raw = np.rad2deg(np.abs(lines[:, 0, 1] - np.pi/2))
print(np.mean(raw[raw < 45]))

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
