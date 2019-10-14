import PIL
from PIL import Image
import numpy as np
import os
import matplotlib.pyplot as plt

def load_velodyne_points(file):
    points_path = file
    points = np.fromfile(points_path, dtype=np.float32).reshape(-1, 4)
    points = points[:, :3]
    return points


def load_image(file):
    im = Image.open(file)
    return im


def create_lidar_image(counter, point_cloud, image: PIL.Image):
    # rate = 2
    # point_cloud = [point for i, point in enumerate(point_cloud) if i % rate == 0]

    point_cloud = [point for point in point_cloud if point[2] >= 3]

    xs = [x[0] / x[2] for x in point_cloud]
    ys = [x[1] / x[2] for x in point_cloud]
    zs = [x[2] for x in point_cloud]

    max_color = max(zs)
    # ((max(0, 255 * z / max_color), ) * 3 + (255, )) for z in zs]
    zs = [(abs(z)) ** 0.5 for z in zs]
    zs = [(1,
           min(1.0, ((abs(z - 3) / max_color) * 30)),
           0)
          for z in zs]

    tuples = [(x, y, z) for x, y, z in zip(xs, ys, zs)]

    image = np.array(image)
    for x, y, z in tuples:
        x, y = map(int, (x, y))
        if x < 0 or x >= image.shape[1]:
            continue

        if y < 0 or y >= image.shape[0]:
            continue

        image[y, x] = tuple(v * 255 for v in z)

    image = PIL.Image.fromarray(np.uint8(image))

    image.save("output/pil-{}.png".format(os.path.basename(counter)))

    # # zs = ['r' for z in zs]
    # plt.imshow(image)
    # plt.xlim(0, 1242)
    # plt.ylim = (-375, 0)
    # plt.scatter(xs, ys, c=zs, s=1)
    # plt.show(block=True)
    # plt.pause(5)
    # # plt.pause(0.1)
    # # plt.close()
    # plt.savefig("output/{}.png".format(os.path.basename(counter)))
    # print(counter)


def do(file):
    points = load_velodyne_points(file)
    image = load_image(file.replace("velodyne_points", "image_02").replace("bin", "png"))

    rotation_matrix = np.fromstring(
        "7.533745e-03 -9.999714e-01 -6.166020e-04 1.480249e-02 7.280733e-04 -9.998902e-01 9.998621e-01 7.523790e-03 1.480755e-02",
        dtype="float32", sep=' ')
    rotation_matrix = np.array(rotation_matrix)
    rotation_matrix = rotation_matrix.reshape((3, 3))

    translation = np.array([[np.fromstring(
        "-4.069766e-03 -7.631618e-02 -2.717806e-01", dtype="float32", sep=' ')]])
    translation = translation.reshape(1, 3)

    big_matrix = np.concatenate(
        (rotation_matrix, np.transpose(translation)), 1)

    intrinsics_matrix = np.fromstring(
        "7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01 0.000000e+00 7.215377e+02 1.728540e+02 2.163791e-01 0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03",
        dtype="float32", sep=' ')
    intrinsics_matrix = intrinsics_matrix.reshape((3, 4))
    intrinsics_matrix = intrinsics_matrix[:, :3]

    points_camera = list()
    for point in points:
        point = np.concatenate((point, np.array([1])))
        point_camera = point
        point_camera = big_matrix.dot(point)
        point_camera = intrinsics_matrix.dot(point_camera)
        points_camera.append(point_camera)

    create_lidar_image(file, points_camera, image)


folder = "/Users/kowalski/Downloads/LIDAR/2011_09_26 2/2011_09_26_drive_0005_sync/"
# folder = "/Users/natalwillisch/Downloads"
files = list()
subfolder = "/velodyne_points/data/"
for file in os.listdir(folder + subfolder):
    if ".bin" in file:
        files.append(folder + subfolder + file)

files.sort()
for file in files:
    print(file)
    do(file)
