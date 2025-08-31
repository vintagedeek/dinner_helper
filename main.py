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
STOP_MODE  = Stop.BRAKE 

# Initial state: motor is stopped
motor_running = False

# Wait time in ms to give cpu a rest and save battery
wait_time_ms = 20
speed_degrees_per_second = 300

def set_led_for_state(running: bool) -> None:
    ev3.light.on(Color.GREEN if running else Color.RED)

def start_motors():
    left_motor.run(speed=speed_degrees_per_second)
    right_motor.run(speed=speed_degrees_per_second)

def stop_motors():
    left_motor.stop(STOP_MODE)
    right_motor.stop(STOP_MODE)

ev3.speaker.say(
    "Press the red button on the front left of vehicle to start or stop the motor"
)
set_led_for_state(motor_running)
while True:
    # Wait until the button is pressed
    while not touch_sensor.pressed():
        wait(wait_time_ms)

    # Wait until the button is released
    while touch_sensor.pressed():
        ev3.speaker.say("Command received")
        wait(wait_time_ms)

    # Toggle the state of the motor
    if motor_running:
        stop_motors()
        motor_running = False
        set_led_for_state(motor_running)
        ev3.speaker.say("Mission complete")
    else:
        start_motors()
        motor_running = True
        set_led_for_state(motor_running)