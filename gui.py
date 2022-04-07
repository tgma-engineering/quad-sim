"""
Class represents gui for visual representation of drone
"""
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import numpy

from drone import Drone
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from quaternion import Quaternion, QuaternionCalculation

class Gui:
    def __init__(self, drone: Drone) -> None:
        plt.ion()
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-10.0, 10.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-10.0, 10.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 5.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation') 
        self.ax.scatter(drone.positionVector[0],drone.positionVector[1],drone.positionVector[2])
        self.fig.show()

    def update(self, drone: Drone):
        plt.pause(0.0000000000000001)
        self.ax.clear()

        drone.update_vertices()

        #v = QuaternionCalculation.calculate_rotation_from_given_quaternion(drone.rotationQuaternion, numpy.array(v)).get_imaginary_part_as_vector()
        #drone.vertices_xAxis = vertices_xAxis
        #drone.vertices_yAxis = vertices_yAxis
        poly = Poly3DCollection(drone.vertices_xAxis, alpha=0.5)
        poly2 = Poly3DCollection(drone.vertices_yAxis, alpha=0.5, color="red")
        self.ax.add_collection3d(poly)
        self.ax.add_collection3d(poly2)
        self.ax.set_xlim3d([-5.0, 5.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-5.0, 5.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 5.0])
        self.ax.set_zlabel('Z')
        self.ax.scatter(drone.positionVector[0],drone.positionVector[1],drone.positionVector[2])
        plt.draw()