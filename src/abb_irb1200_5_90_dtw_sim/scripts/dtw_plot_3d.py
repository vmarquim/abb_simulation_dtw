import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dtw import *   


def dtw_plot_3d(ax, reference_data_frame, query_data_frame):

    print(reference_data_frame)
    print(query_data_frame)

    x1 = reference_data_frame['x'].to_numpy()
    y1 = reference_data_frame['y'].to_numpy()
    z1 = reference_data_frame['z'].to_numpy()

    x2 = query_data_frame['x'].to_numpy()
    y2 = query_data_frame['y'].to_numpy()
    z2 = query_data_frame['z'].to_numpy()

    fig = plt.figure()

    ax.plot3D(x1, y1, z1, color="blue", label="Simulated Trajectory")
    ax.plot3D(x2, y2, z2, color="red", label="Theoretical Trajectory")

    df1 = np.vstack((x1, y1, z1)).reshape(-1, 3)
    df2 = np.vstack((x2, y2, z2)).reshape(-1, 3)

    alignment = dtw(df1, df2, keep_internals=True)

    dtw_axes = alignment.plot("threeway")

    for i, j in zip(alignment.index1, alignment.index2):
        ax.plot3D(np.array([x1[i], x2[j]]), np.array([y1[i], y2[j]]), np.array([z1[i], z2[j]]), linestyle="dashed", linewidth=0.3, color="gray")
        ax.scatter3D(np.array([x1[i], x2[j]]), np.array([y1[i], y2[j]]), np.array([z1[i], z2[j]]), color="gray")

    ax.set_title("DTW - Normalised Distance: %s" % round(alignment.normalizedDistance, 2))
    ax.set_xlabel("x in meters")
    ax.set_ylabel("y in meters")
    ax.set_zlabel("z in meters")
    ax.legend()
    
    return dtw_axes


def plot_circle():
    r = 1
    n = 100
    thetas = np.linspace(0, 2*math.pi, n)
    dtheta = 2*math.pi/n
    x_start = 2*r
    y_start = 0
    x = [x_start]
    y = [y_start]
    for index, theta in enumerate(thetas):
        dx = -r*math.sin(theta)*dtheta
        dy = r*math.cos(theta)*dtheta
        x.append(x[index]+dx)
        y.append(y[index]+dy)

    fig, ax = plt.subplots()
    ax.plot(x, y)
    plt.show()    


def plot_2d(reference_data_frame):
    y1 = reference_data_frame['y'].to_numpy()
    z1 = reference_data_frame['z'].to_numpy()
    fig, ax = plt.subplots()
    ax.plot(y1, z1)
    plt.show()

def plot_dtw_3d_test(axes):

    # Test data
    n1 = 100
    x1 = np.linspace(0, 5, n1)
    y1 = x1*x1
    z1 = np.full_like(x1, 0)

    n2 = n1
    x2 = np.linspace(0, 5, n2)
    y2 = x2*x2
    z2 = np.full_like(x2, 3)

    fig = plt.figure()

    axes.plot3D(x1, y1, z1, color="red", label="Simulated Trajectory")
    axes.plot3D(x2, y2, z2, color="blue", label="Theoretical Trajectory")

    df1 = np.vstack((x1, y1, z1)).reshape(-1, 3)
    df2 = np.vstack((x2, y2, z2)).reshape(-1, 3)

    # print(df1.shape)
    # print(df2.shape)

    alignment = dtw(df1, df2, keep_internals=True)

    # print(alignment.index1)
    # print(alignment.index2)

    # print(x1)


    # print(alignment.__dict__)

    for i, j in zip(alignment.index1, alignment.index2):
        # print(i, j)
        axes.plot3D(np.array([x1[i], x2[j]]), np.array([y1[i], y2[j]]), np.array([z1[i], z2[j]]), linestyle="dashed", linewidth=0.3, color="gray")
        axes.scatter3D(np.array([x1[i], x2[j]]), np.array([y1[i], y2[j]]), np.array([z1[i], z2[j]]), color="gray")

    axes.set_title("DTW - Normalised Distance: %s" % round(alignment.normalizedDistance, 2))
    axes.set_xlabel("x in meters")
    axes.set_ylabel("y in meters")
    axes.set_zlabel("z in meters")

    axes.legend()

    dtw_axes = alignment.plot("threeway")
    # plt.show()

    return dtw_axes


if __name__ == "__main__":
    plot_circle()