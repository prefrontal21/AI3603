import matplotlib.pyplot as plt
import sys,os
sys.path.append(os.path.realpath('../AI3603_HW1'))

from hybrid_a_star import hybrid_a_star
def main():
    print(__file__ + " start!!")

    # start and goal position
    sx, sy, stheta = 0, 0, 0
    gx, gy, gtheta = 4, 4, 0

    current_map = [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0]

    ]

    ox, oy = [], []
    for i in range(6):
        for j in range(6):
            if current_map[i][j]:
                ox.append(i)
                oy.append(j)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    hy_a_star = hybrid_a_star(-6, 6, -6, 6, current_map=current_map,
                              resolution=1, vehicle_length=2)
    path, theta = hy_a_star.find_path((sx, sy, stheta), (gx, gy, gtheta))

    rx, ry = [], []
    for node in path:
        rx.append(node[0])
        ry.append(node[1])

    plt.plot(rx, ry, "-r")
    plt.savefig("out/test_hybrid_A_star.png")


if __name__ == '__main__':
    main()
