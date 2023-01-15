import cv2
import numpy as np
import sys

sys.setrecursionlimit(10**6)

class DFS():

    @staticmethod
    def get_paths(graph, start, end, path=[]):
        path = path + [start]
        if start == end:
            return [path]
        if not start in graph.keys():
            return []
        paths = []
        for node in graph[start].keys():
            if node not in path and node != 'case':
                newpaths = DFS.get_paths(graph, node, end, path)
                for newpath in newpaths:
                    paths.append(newpath)
        return paths


class bot_pathplanner():

    def __init__(self):
        self.DFS = DFS()

    @staticmethod
    def cords_to_pts(cords):
        return [cord[::-1] for cord in cords]

    def draw_path_on_maze(self, maze, shortest_path_pts, method):

        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        self.choosen_route = np.zeros_like(maze_bgr)

        rang = list(range(0, 254, 25))

        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            # Blue : []   [0 1 2 3 251 255 251 3 2 1 0] 0-depthperc-0
            # Green :[]     depthperc
            # Red :  [] 100-depthperc
            color = (
                int(255 * (abs(per_depth+(-1*(per_depth > 0.5)))*2)),
                int(255 * per_depth),
                int(255 * (1-per_depth))
            )
            cv2.line(
                maze_bgr, shortest_path_pts[i], shortest_path_pts[i+1], color)
            cv2.line(self.choosen_route,
                     shortest_path_pts[i], shortest_path_pts[i+1], color, 3)

        img_str = "maze (Found Path) [" + method + "]"

        cv2.namedWindow(img_str, cv2.WINDOW_FREERATIO)
        cv2.imshow(img_str, maze_bgr)
        self.img_shortest_path = maze_bgr.copy()

    def find_path_nd_display(self, graph, start, end, maze, method="DFS"):
        if method == "DFS":
         # type: ignore
            paths = self.DFS.get_paths(graph, start, end)
            path_to_display = paths[0]

        pathpts_to_display = self.cords_to_pts(path_to_display)
        self.draw_path_on_maze(maze, pathpts_to_display, method)
        cv2.waitKey(0)
