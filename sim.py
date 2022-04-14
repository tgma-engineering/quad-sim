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
#drone.set_motor_speed(2,2,2,2)
physics = Physics(drone)

startTime = datetime.datetime.now()

starttTime = datetime.datetime.now()

counter = 0

def toggle_motor_speed(drone, bool):
    if bool:
        drone.set_motor_speed(2,2,2,2)
    else:
        drone.set_motor_speed(0,0,0,0)

# starting loop
flag = True
while True:
    currentTime = datetime.datetime.now()
    delta_time = (currentTime - startTime).total_seconds()
    startTime = currentTime
    gui.update(drone)
    physics.calculatePosition(delta_time)
    counter = counter + 1
    #print("counter: " + str(counter))
    if counter > 10:
        #exit()
        #print((datetime.datetime.now() - starttTime).total_seconds())
        counter = 0
        #drone.set_motor_speed(0,0,0,0)
        toggle_motor_speed(drone, flag)
        flag = not flag