import cv2
import numpy as np

draw_intrstpts = False


class bot_mapper():

    def __init__(self):
        self.graphified = False
        self.crp_amnt = 5

    @staticmethod
    def get_surrounding(maze, row, col):
        maze = cv2.threshold(maze, 0, 1, cv2.THRESH_BINARY)[1]

        rows = maze.shape[0]
        cols = maze.shape[1]

        top_row = False
        bottom_row = False
        left_col = False
        right_col = False

        if row == 0:
            top_row = True

        if row == rows-1:
            bottom_row = True

        if col == 0:
            left_col = True

        if col == cols-1:
            right_col = True

        if top_row or left_col:
            top_left = 0
        else:
            top_left = maze[row-1][col-1]

        if top_row or right_col:
            top_right = 0
        else:
            top_right = maze[row-1][col+1]

        if bottom_row or left_col:
            bottom_left = 0
        else:
            bottom_left = maze[row+1][col-1]

        if bottom_row or right_col:
            bottom_right = 0
        else:
            bottom_right = maze[row+1][col+1]

        if top_row:
            top = 0
        else:
            top = maze[row-1][col]

        if bottom_row:
            bottom = 0
        else:
            bottom = maze[row+1][col]

        if left_col:
            left = 0
        else:
            left = maze[row][col-1]

        if right_col:
            right = 0
        else:
            right = maze[row][col+1]

        no_pathways = (top_left + top_right + bottom_left +
                       bottom_right + top + bottom + left + right)

        if no_pathways > 2:
            print("  [ top_left , top      , top_rgt  ,lft    , rgt      , btm_left , btm      , btm_rgt   ] \n [ ", str(top_left), " , ", str(top), " , ", str(
                top_right), " ,\n   ", str(left), " , ", "-", " , ", str(right), " ,\n   ", str(bottom_left), " , ", str(bottom), " , ", str(bottom_right), " ] ")
            print("\nno_of_pathways [row,col]= [ ",
                  row, " , ", col, " ] ", no_pathways)

        return top_left, top, top_right, right, bottom_right, bottom, bottom_left, left, no_pathways

    @staticmethod
    def triangle(image, ctr_pt, radius, colour=(0, 255, 255), thickness=2):
        # Polygon corner points coordinates
        pts = np.array([[ctr_pt[0], ctr_pt[1]-radius],
                        [ctr_pt[0]-radius, ctr_pt[1]+radius],
                        [ctr_pt[0]+radius, ctr_pt[1]+radius]
                        ], np.int32
                       )
        pts = pts.reshape((-1, 1, 2))

        image = cv2.polylines(image, [pts], True, colour, thickness)
        return image

    def one_pass(self, maze):
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)

        cv2.namedWindow('interest points', cv2.WINDOW_FREERATIO)
        rows = maze.shape[0]
        cols = maze.shape[1]

        for row in range(rows):
            for col in range(cols):
                if maze[row][col] == 255:
                    top_left, top, top_right, right, bottom_right, bottom, bottom_left, left, paths = self.get_surrounding(
                        maze.copy(), row, col)

                if row == 0 or col == 0 or row == rows-1 or col == cols-1:
                    if row == 0 and col == 0:
                        maze_bgr[row][col] = [0, 128, 255]
                        cv2.imshow('interest points', maze_bgr)
                    else:
                        maze_bgr[row][col] = [0, 128, 255]
                        cv2.imshow('interest points', maze_bgr)

                elif (paths == 1):
                    crop = maze[row-1:row+2, col-1:col+2]
                    print(" ** [Dead End] ** \n", crop)
                    maze_bgr[row][col] = (0, 0, 255)  # Red color
                    if draw_intrstpts:
                        maze_bgr = cv2.circle(
                            maze_bgr, (col, row), 10, (0, 0, 255), 4)
                    cv2.imshow("Maze (Interest Points)", maze_bgr)

                # Check if it is either a *Turn* or just an ordinary path
                elif (paths == 2):
                    crop = maze[row-1:row+2, col-1:col+2]
                    nzero_loc = np.nonzero(crop > 0)
                    nzero_ptA = (nzero_loc[0][0], nzero_loc[1][0])
                    nzero_ptB = (nzero_loc[0][2], nzero_loc[1][2])
                    if not (((2 - nzero_ptA[0]) == nzero_ptB[0]) and
                            ((2 - nzero_ptA[1]) == nzero_ptB[1])):
                        maze_bgr[row][col] = (255, 0, 0)
                        cv2.imshow("Maze (Interest Points)", maze_bgr)
                # Check if it is either a *3-Junc* or a *4-Junc*
                elif (paths > 2):
                    if (paths == 3):
                        maze_bgr[row][col] = (255, 244, 128)
                        if draw_intrstpts:
                            maze_bgr = self.triangle(
                                maze_bgr, (col, row), 10, (144, 140, 255), 4)
                            cv2.imshow("Maze (Interest Points)", maze_bgr)
                    else:
                        maze_bgr[row][col] = (128, 0, 128)
                        if draw_intrstpts:
                            cv2.rectangle(maze_bgr, (col-10, row-10),
                                          (col+10, row+10), (255, 215, 0), 4)
                        cv2.imshow("Maze (Interest Points)", maze_bgr)

    def graphify(self, extracted_maze):
        if not self.graphified:
            cv2.imshow('extracted_maze', extracted_maze)

            thinned = cv2.ximgproc.thinning(extracted_maze, extracted_maze)
            cv2.imshow('thinned', thinned)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
            thinned_dilated = cv2.dilate(thinned, kernel)
            _, bw2 = cv2.threshold(
                thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            thinned = cv2.ximgproc.thinning(bw2)
            cv2.imshow('thinned', thinned)

            thinned_crop = thinned[self.crp_amnt:thinned.shape[0]-self.crp_amnt,
                                   self.crp_amnt:thinned.shape[1]-self.crp_amnt]

            cv2.imshow('thinned_crop', thinned_crop)

            extracted_maze_cropped = extracted_maze[self.crp_amnt:extracted_maze.shape[0]-self.crp_amnt,
                                                    self.crp_amnt:extracted_maze.shape[1]-self.crp_amnt]

            extracted_maze_cropped = cv2.cvtColor(
                extracted_maze_cropped, cv2.COLOR_GRAY2BGR)
            extracted_maze_cropped[thinned_crop > 0] = [0, 255, 255]
            cv2.imshow('extracted_maze_cropped', extracted_maze_cropped)

            cv2.waitKey(0)
