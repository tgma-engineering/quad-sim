'''
Test
'''
import datetime
import time

from drone import Drone
from gui import Gui
from physics import Physics
from control import Controller
import numpy as np

if __name__ == '__main__':
    pass

drone = Drone()
gui = Gui(drone) 
#drone.set_motor_speed(2,2,2,2)
physics = Physics(drone)
control = Controller()

startTime = datetime.datetime.now()

starttTime = datetime.datetime.now()

# starting loop
flag = True
while True:
    currentTime = datetime.datetime.now()
    delta_time = (currentTime - startTime).total_seconds()
    startTime = currentTime
    gui.update(drone)
    x_r = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                   dtype=np.float64)  # Desired state, this one is a good start
    x = np.concatenate((drone.positionVector, drone.velocityVector,
                        drone.rotationQuaternion.get_as_numpy_arr(),
                        drone.angularVelocity.get_as_numpy_arr()))
    u = control.control(x, x_r)
    u = np.clip(u, 0, 4)
    drone.set_motor_speed(u[0], u[1], u[2], u[3])
    physics.calculatePosition(delta_time)