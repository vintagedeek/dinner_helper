#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
ev3 = EV3Brick()
right_motor = Motor(Port.C)
left_motor = Motor(Port.B)
touch_sensor = TouchSensor(Port.S1)
color_sensor = ColorSensor(Port.S3)


# Initial state: motor is stopped
motor_running = False

# Wait time in ms to give cpu a rest and save battery
wait_time_ms = 20

ev3.speaker.say(
    "Press the red button on the front left of vehicle to start or stop the motor"
)
while True:
    # Wait until the button is pressed
    while not touch_sensor.pressed():
        wait(0)

    # Wait until the button is released
    while touch_sensor.pressed():
        ev3.speaker.say("Command received")
        wait(0)

    # Toggle the state of the motor
    if motor_running:
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        ev3.speaker.say("Mission complete")
        motor_running = False
    else:
        left_motor.run(speed=300)
        right_motor.run(speed=300)
        motor_running = True