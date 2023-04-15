import cv2
import math
import sys
import numpy as np

import sys


class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)

    def construct_graph(self, nodes, init_graph):
        graph = {}
        for node in nodes:
            graph[node] = {}

        graph.update(init_graph)

        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value

        return graph

    def get_nodes(self):
        return self.nodes

    def get_outgoing_edges(self, node):
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections

    def value(self, node1, node2):
        return self.graph[node1][node2]


def dijkstra_algorithm(graph, start_node):
    unvisited_nodes = list(graph.get_nodes())

    shortest_path = {}

    previous_nodes = {}

    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    shortest_path[start_node] = 0

    while unvisited_nodes:
        current_min_node = None
        for node in unvisited_nodes:
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node

        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                previous_nodes[neighbor] = current_min_node

        unvisited_nodes.remove(current_min_node)

    return previous_nodes, shortest_path


def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node

    while node != start_node:
        path.append(node)
        try:
            node = previous_nodes[node]
        except KeyError:
            return "SUCK MY DICK, BITCH"

    path.append(start_node)
    return shortest_path[target_node], path[::-1]


def bresenham_line(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    err = dx - dy
    res = []
    while True:
        res.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return res

center = []
delta = []
keys = []
dict=[]
nodes = [0, 1, 2, 3]
mask1 = [0, 35, 40]
init_graph = {}

cv2.namedWindow("image")
img = cv2.imread("aba1.png")
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

m = []

for i in range(len(center)):
    m.append([])

print(center)
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
sum, path = print_result(previous_nodes, shortest_path, start_node=0, target_node=3)

print(sum, path)

for i in range(len(path) - 1):
    cv2.line(img, (center[path[i]][0], center[path[i]][1]), (center[path[i+1]][0], center[path[i+1]][1]), (255, 0, 0), 1)

while cv2.waitKey(1) != 27:

    cv2.imshow('image', img)