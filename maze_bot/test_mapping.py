import cv2

from bot_mapping import bot_mapper

bot_mapper_ = bot_mapper()


def main():

    tiny = cv2.imread(
        "/home/jacob/Desktop/party_tiny.png", cv2.IMREAD_GRAYSCALE)

    # Displaying Tiny Maze
    cv2.namedWindow("tiny_maze", cv2.WINDOW_FREERATIO)
    cv2.imshow("tiny_maze", tiny)

    # [Mapping] Applying the One Pass Algorithm to Convert Maze Image to Graph
    bot_mapper_.one_pass(tiny)
    print("** Graph Extracted **\n")
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
