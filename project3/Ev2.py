#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos

# Draw some obstacles
def plotObstacles(ax):

    ax.scatter(-0.7, 0.25, color='red', s=100, label='Start')
    ax.scatter(-0.9, 0.9, color='red', s=100, label='Goal')
    ax.text(-0.7, 0.25 + 0.1, "Start", horizontalalignment='center')
    ax.text(-0.9, 0.9 + 0.1, "Goal", horizontalalignment='center')


    ax.add_patch(patches.Polygon([(-0.45,0.2),(-0.45,-0.95),(0.25,-0.95),(0.25,0.2)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(-0.9,0.6),(-0.9,0.5),(0.5,0.5),(0.5,0.6)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(0.4,0.6),(0.4,-0.2),(0.5,-0.2),(0.5,0.6)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(-0.9,0.5),(-0.9,-1.3),(-0.8,-1.3),(-0.8,0.5)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(-0.9,-1.3),(-0.9,-1.2),(0.0,-1.2),(0.0,-1.3)], fill=True, color='0.20'))


# Plot only the obstacles, no paths
def plotR2(path):
    fig = plt.figure()
    ax = fig.gca()

    plotObstacles(ax)  # Plot obstacles only, no path

    plt.show()

def plotSE2(path):
    fig = plt.figure()
    ax = fig.gca()

    plotObstacles(ax)  # Plot obstacles only, no path

    plt.axis([-2, 2, -2, 2])
    plt.show()

def plotWeird(path):
    fig = plt.figure()
    ax = fig.gca()

    # Translate configuration space path into SE(2) path
    path = [[cos(p[1]) * p[0] - 2, sin(p[1]) * p[0] - 2, p[1]] for p in path]

    plotObstacles(ax)  # Plot obstacles only, no path

    plt.show()

# Read the cspace definition and the path from filename
def readPath(filename):
    lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]

    if len(lines) == 0:
        print("That file's empty!")
        sys.exit(1)

    cspace = lines[0].strip()
    if (cspace != 'R2' and cspace != 'SE2' and cspace!= 'Weird'):
        print("Unknown C-space Identifier:" + cspace)
        sys.exit(1)

    data = [[float(x) for x in line.split(' ')] for line in lines[1:]]
    return cspace, data

if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'path2.txt'

    cspace, path = readPath(filename)

    if cspace == 'R2':
        plotR2(path)
    elif cspace == 'SE2':
        plotSE2(path)
    elif cspace == 'Weird':
        plotWeird(path)
