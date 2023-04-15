import cv2
import math
import sys
import numpy as np
from graphclass import Graph
from algos import dijkstra_algorithm
from algos import bresenham_line
from algos import result



center = []
nodes = [0, 1, 2, 3]
mask1 = [0, 35, 40]
init_graph = {}

cv2.namedWindow("image")
img = cv2.imread("aba2.png")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
h_min = np.array(mask1, np.uint8)
h_max = np.array((255, 255, 255), np.uint8)
thresh = cv2.inRange(hsv, h_min, h_max)
contours_1, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

for node in nodes:
    init_graph[node] = {}

for i in contours_1:
    M = cv2.moments(i)
    x1 = int(M['m10'] / M['m00'])
    y1 = int(M['m01'] / M['m00'])
    center.append([x1, y1])


for i in range(len(center)):
    cv2.putText(img, f"{i}", (center[i][0]+20, center[i][1]-20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 1)

for i in range(len(center)):
    for j in range(i+1, len(center)):
        mat = bresenham_line(center[i][0], center[i][1], center[j][0], center[j][1])
        for k in mat:
            if img[k[1]][k[0]][0] == 0 and img[k[1]][k[0]][1] == 0 and img[k[1]][k[0]][2] == 0:
                break
        else:
            cv2.line(img, (center[i][0], center[i][1]), (center[j][0], center[j][1]), (0, 255, 0), 1)

            init_graph[i][j] = math.dist(center[i], center[j])


graph = Graph(nodes, init_graph)
previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node=0)
sum, path = result(previous_nodes, shortest_path, start_node=0, target_node=3)

print(f" Минимальное расстояние: {sum/100} м", "\n", f"Путь: {path}")

for i in range(len(path) - 1):
    cv2.line(img, (center[path[i]][0], center[path[i]][1]), (center[path[i+1]][0], center[path[i+1]][1]), (255, 0, 0), 1)

while cv2.waitKey(1) != 27:
    cv2.imshow('image', img)