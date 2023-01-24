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
        self.dijisktra =dijisktra()

        self.path_to_goal = []
        self.img_shortest_path = []

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

        elif method == "dijkstra":
            if not self.dijisktra.shortestpath_found  :
                self.dijisktra.find_best_routes(graph, start, end)

            path_to_display = self.dijisktra.shortest_path

        pathpts_to_display = self.cords_to_pts(path_to_display)
        self.path_to_goal = pathpts_to_display
        self.draw_path_on_maze(maze, pathpts_to_display, method)
        # cv2.waitKey(0)

class Heap():

    def __init__(self):
        # Priority queue will be stored in an array (list of list containing vertex and their resp distance)
        self.array = []
        # Counter to track nodes left in priority queue
        self.size = 0
        # Curr_pos of each vertex is stored
        self.posOfVertices = []

    # create a minheap node => List(vertex,distance)
    def new_minHeap_node(self, v, dist):
        return([v, dist])

    # Swap node a (List_A) with node b (List_b)
    def swap_nodes(self, a, b):
        temp = self.array[a]
        self.array[a] = self.array[b]
        self.array[b] = temp

    # Convert any part of complete tree in minHeap in O(nlogn) time
    def minHeapify(self, node_idx):
        smallest = node_idx
        left = (node_idx*2)+1
        right = (node_idx*2)+2

        if ((left < self.size) and (self.array[left][1] < self.array[smallest][1])):
            smallest = left
        if ((right < self.size) and (self.array[right][1] < self.array[smallest][1])):
            smallest = right

        # If node_idx is not the smallest
        if(smallest != node_idx):
            # Update the positions to keep smallest on top
            self.posOfVertices[self.array[node_idx][0]] = smallest
            self.posOfVertices[self.array[smallest][0]] = node_idx
            # Swap node_idx with smallest
            self.swap_nodes(node_idx, smallest)
            # Recursively call minHeapify until all subnodes part of minheap or no more subnodes left
            self.minHeapify(smallest)

    # extract top (min value) node from the min-heap => then minheapify to keep heap property
    def extractmin(self):

        # Handling boudary condtion
        if self.size == 0:
            return

        # root node (list[root_vertex,root_value]) extracted
        root = self.array[0]

        # Move Last node to top
        lastNode = self.array[self.size-1]
        self.array[0] = lastNode

        # Update the postion of vertices
        self.posOfVertices[root[0]] = self.size-1
        self.posOfVertices[lastNode[0]] = 0

        # Decrease the size of minheap by 1
        self.size -= 1

        # Perform Minheapify from root
        self.minHeapify(0)
        # Return extracted root node to user
        return root

    # Update distance for a node to a new found shorter distance
    def decreaseKey(self, vertx, dist):

        # retreviing the idx of vertex we want to decrease value of
        idxofvertex = self.posOfVertices[vertx]

        self.array[idxofvertex][1] = dist

        # Travel up while complete heap is not heapified
        # While idx is valid and (Updated_key_dist < Parent_key_dist)
        while((idxofvertex > 0) and (self.array[idxofvertex][1] < self.array[(idxofvertex-1)//2][1])):
            # Update position of parent and curr_node
            self.posOfVertices[self.array[idxofvertex][0]] = (idxofvertex-1)//2
            self.posOfVertices[self.array[(idxofvertex-1)//2][0]] = idxofvertex

            # Swap curr_node with parent
            self.swap_nodes(idxofvertex, (idxofvertex-1)//2)

            # Navigate to parent and start process again
            idxofvertex = (idxofvertex-1)//2

    # A utility function to check if a given
    # vertex 'v' is in min heap or not
    def isInMinHeap(self, v):

        if self.posOfVertices[v] < self.size:
            return True
        return False


class dijisktra():

    def __init__(self):

        # State variable
        self.shortestpath_found = False
        # Once found save the shortest path
        self.shortest_path = []

        self.shortest_path_overlayed = []

        # instance variable assigned obj of heap class for implementing required priority queue
        self.minHeap = Heap()

        # Creating dictionaries to manage the world
        self.idxs2vrtxs = {}
        self.vrtxs2idxs = {}
        # Counter added to track total nodes visited to
        #               reach goal node
        self.dijiktra_nodes_visited = 0

    def ret_shortestroute(self, parent, start, end, route):

        # Keep updating the shortest route from end to start by visiting closest vertices starting fron end
        route.append(self.idxs2vrtxs[end])

        # Once we have reached the start (maze_entry) => Stop! We found the shortest route
        if (end == start):
            return

        # Visit closest vertex to each node
        end = parent[end]
        # Recursively call function with new end point until we reach start
        self.ret_shortestroute(parent, start, end, route)

    def find_best_routes(self, graph, start, end):

        # Teaking the first item of the list created by list comprehension
        # Which is while looping over the key value pair of graph. Return the pairs_idx that match the start key
        start_idx = [idx for idx, key in enumerate(
            graph.items()) if key[0] == start][0]
        print("Index of search key : {}".format(start_idx))

        # Distanc list storing dist of each node
        dist = []
        # Storing found shortest subpaths [format ==> (parent_idx = closest_child_idx)]
        parent = []

        # Set size of minHeap to be the total no of keys in the graph.
        self.minHeap.size = len(graph.keys())

        for idx, v in enumerate(graph.keys()):

            # Initialize dist for all vertices to inf
            dist.append(1e7)
            # Creating BinaryHeap by adding one node([vrtx2idx(v),dist]) at a time to minHeap Array
            # So instead of vertex which is a tuple representing an Ip we pass in an index
            self.minHeap.array.append(
                self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.posOfVertices.append(idx)

            # Initializing parent_nodes_list with -1 for all indices
            parent.append(-1)
            # Updating dictionaries of vertices and their positions
            self.vrtxs2idxs[v] = idx
            self.idxs2vrtxs[idx] = v

        # Set dist of start_idx to 0 while evertything else remains Inf
        dist[start_idx] = 0
        # Decrease Key as new found dist of start_vertex is 0
        self.minHeap.decreaseKey(start_idx, dist[start_idx])

        # Loop as long as Priority Queue has nodes.
        while(self.minHeap.size != 0):

            # Counter added for keeping track of nodes visited to reach goal node
            self.dijiktra_nodes_visited += 1

            # Retrieve the node with the min dist (Highest priority)
            curr_top = self.minHeap.extractmin()
            u_idx = curr_top[0]
            u = self.idxs2vrtxs[u_idx]

            # check all neighbors of vertex u and update their distance if found shorter
            for v in graph[u]:
                # Ignore Case node
                if v != "case":

                    print("Vertex adjacent to {} is {}".format(u, v))
                    v_idx = self.vrtxs2idxs[v]

                    # if we have not found shortest distance to v + new found dist < known dist ==> Update dist for v
                    if (self.minHeap.isInMinHeap(v_idx) and (dist[u_idx] != 1e7) and
                       ((graph[u][v]["cost"] + dist[u_idx]) < dist[v_idx])):

                        dist[v_idx] = graph[u][v]["cost"] + dist[u_idx]
                        self.minHeap.decreaseKey(v_idx, dist[v_idx])
                        parent[v_idx] = u_idx

            # End Condition: When our End goal has already been visited.
            #                This means shortest part to end goal has already been found
            #     Do   --->              Break Loop
            if (u == end):
                break

        shortest_path = []
        self.ret_shortestroute(
            parent, start_idx, self.vrtxs2idxs[end], shortest_path)

        # Return route (reversed) to start from the beginned
        self.shortest_path = shortest_path[::-1]
        self.shortestpath_found = True
