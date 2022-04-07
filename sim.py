'''
Test
'''
import datetime
import time

from drone import Drone
from gui import Gui
from physics import Physics

if __name__ == '__main__':
    pass

drone = Drone()
gui = Gui(drone) 
drone.set_motor_speed(2,3,2,3)
physics = Physics(drone)

startTime = datetime.datetime.now()

starttTime = datetime.datetime.now()

counter = 0

# starting loop
while True:
    currentTime = datetime.datetime.now()
    delta_time = (currentTime - startTime).total_seconds()
    startTime = currentTime
    gui.update(drone)
    physics.calculatePosition(delta_time)
    counter = counter + 1
    #print("counter: " + str(counter))
    if counter > 3:
        #exit()
        #print((datetime.datetime.now() - starttTime).total_seconds())
        drone.set_motor_speed(1,2,1,1)

