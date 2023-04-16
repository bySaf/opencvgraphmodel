import cv2
import math
import sys
import time
from datetime import datetime
import numpy as np
from graphclass import Graph
from algos import dijkstra_algorithm
from algos import bresenham_line
from algos import result

centers = []
init_graph = {}
coordinates = []
position_number = 0
sum1 = 0
sumtime = 0
run = True
mask1 = [0, 35, 40]
filename = "test1.png"
cv2.namedWindow("image")
img = cv2.imread(filename)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
h_min = np.array(mask1, np.uint8)
h_max = np.array((255, 255, 255), np.uint8)
thresh = cv2.inRange(hsv, h_min, h_max)
contours_1, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

for i in contours_1:
    M = cv2.moments(i)
    x1 = int(M['m10'] / M['m00'])
    y1 = int(M['m01'] / M['m00'])
    centers.append([x1, y1])

n = len(centers)
nodes = [--x for x in range(0, n)]

for i in range(len(centers)):
    cv2.putText(img, f"{i}", (centers[i][0] + 20, centers[i][1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 1)

for node in nodes:
    init_graph[node] = {}

for i in range(n):
    for j in range(i + 1, n):
        mat = bresenham_line(centers[i][0], centers[i][1], centers[j][0], centers[j][1])
        for k in mat:
            if img[k[1]][k[0]][0] == 0 and img[k[1]][k[0]][1] == 0 and img[k[1]][k[0]][2] == 0:
                break
        else:
            cv2.line(img, (centers[i][0], centers[i][1]), (centers[j][0], centers[j][1]), (0, 255, 0), 1)

            init_graph[i][j] = math.dist(centers[i], centers[j])

graph = Graph(nodes, init_graph)
previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node=0)

try:
    sum, path = result(previous_nodes, shortest_path, start_node=0, target_node=3)
    print(f" Минимальное расстояние: {sum / 100} м", "\n", f"Путь: {path}")

    for i in range(len(path) - 1):
        cv2.line(img, (centers[path[i]][0], centers[path[i]][1]), (centers[path[i + 1]][0], centers[path[i + 1]][1]),
                 (255, 0, 0), 1)
    cv2.imwrite(f"output{filename[-5]}.png", img)




except KeyError:
    my_file = open(f"output{filename[-5]}", "w+")
    my_file.write("Невозможно")
    print("Невозможно")
    my_file.close()
    raise SystemExit

for i in path:
    coordinates.append((round(centers[i][0] / 100, 2), round(4.8 - centers[i][1] / 100, 2), 2.00))

print(coordinates)


def callback(event):
    global ap
    global run
    global coordinates
    global position_number
    global sum
    global sum1
    global sumtime
    global current_time
    global current_time1

    event = event.data
    if event == CallbackEvent.ENGINES_STARTED:
        ap.takeoff()
    elif event == CallbackEvent.TAKEOFF_COMPLETE:
        position_number = 0
        ap.goToLocalPoint(coordinates[position_number][0], coordinates[position_number][1],
                          coordinates[position_number][2])
        position_number += 1
        current_time = datetime.now().time()

    elif event == CallbackEvent.POINT_REACHED:
        current_time1 = datetime.now().time()
        x = math.dist((coordinates[position_number - 1][0], position_number[position_number - 1][1]),
                      (coordinates[position_number][0], position_number[position_number][1]))
        s = current_time1.hour * 3600 + current_time1.minute * 60 + current_time1.second
        s1 = current_time.hour * 3600 + current_time.minute * 60 + current_time.second
        sum-=x
        sum1+=x
        print(f"Была достигнута {position_number} точка. Было пройдено {sum1} м, осталось {sum}. Предположительно осталось {sum/(s-s1)} c")
        position_number += 1
        current_time = datetime.now().time()
        if position_number < len(coordinates):
            current_time = datetime.now().time()
            ap.goToLocalPoint(coordinates[position_number][0], coordinates[position_number][1],
                              coordinates[position_number][2])

        else:
            ap.landing()
    elif event == CallbackEvent.COPTER_LANDED:
        print("finish programm")
        run = False


board = BoardManager()
ap = FlightController(callback)

once = False

while not rospy.is_shutdown() and run:
    if board.runStatus() and not once:  # проверка подлкючения RPi к Пионеру
        print("start programm")
        ap.preflight()  # отдаем команду выполенения предстартовой подготовки
        once = True
    pass
