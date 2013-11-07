#!/usr/bin/env python

import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from numpy import zeros
from numpy import ones

import time
import os
import sys
import threading
import random

class Hello(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        os.system("./a.out >> output.txt")

grid = None

def draw_grid():
    # This assumes you are using a numpy array for your grid
    width, height = grid.shape
    # glRasterPos2f(-1, -1)
    glDrawPixels(width, height, GL_LUMINANCE, GL_FLOAT, grid)
    glFlush()
    glutSwapBuffers()

def update_grid(new_grid):
    global grid
    grid = new_grid



def init_window(width, height):
    global window
    global grid
    grid = zeros((width, height))
    glutInit(())
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(width, height)
    glutInitWindowPosition(0, 0)
    window = glutCreateWindow("Grid filter")
    glutDisplayFunc(draw_grid)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    # glutMainLoop()

def readfile(g):
    try:
        f = open('output.txt', 'r')

        for i in range(800):
            line = f.readline()
            arr = line.split()
            for j in range(800):
                g[i][j] = arr[j]
        end = f.readline().strip()
        return True
    except:
        return False

if __name__ == '__main__':

    # h = Hello()
    # h.start()
    # time.sleep(0.5)

    init_window(800,800)
    grid2 = ones((800,800))

    while True:
        # read the file and populate the grid
        read = False
        while not read:
            read = readfile(grid2)            
        update_grid(grid2)
        draw_grid()
        time.sleep(1)