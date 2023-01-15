import cv2

from bot_mapping import bot_mapper
from bot_planning import DFS

bot_mapper_ = bot_mapper()
DFS_ = DFS()


def main():

    tiny = cv2.imread(
        "/home/jacob/Desktop/party_tiny.png", cv2.IMREAD_GRAYSCALE)

    # Displaying Tiny Maze
    cv2.namedWindow("tiny_maze", cv2.WINDOW_FREERATIO)
    cv2.imshow("tiny_maze", tiny)

    # [Mapping] Applying the One Pass Algorithm to Convert Maze Image to Graph
    bot_mapper_.one_pass(tiny)
    print("** Graph Extracted **\n")
    # bot_mapper_.Graph.display_graph()
    # print("\n** Graph Displayed **\n")
    cv2.waitKey(0)

    # [Planning] Applying the Depth First Search Algorithm to Find Shortest Path
    start = bot_mapper_.Graph.start
    end = bot_mapper_.Graph.end

    print("Paths from {} to {}".format(start, end))

    paths = DFS_.get_paths(bot_mapper_.Graph.graph, start, end)

    print("Paths from {} to {} is : \n {}".format(start, end, paths))


if __name__ == '__main__':
    main()
