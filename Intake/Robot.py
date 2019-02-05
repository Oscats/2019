#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import wpilib.drive 

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        # motors
        self.left_motor = wpilib.Victor(0)
        self.right_motor = wpilib.Victor(1) 
        
        # stick & timer
        self.stick = wpilib.Joystick()
        self.timer = wpilib.Timer()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        print ("auto")


    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
       # motor command
        self.left_motor.set(-1*(self.stick.getY())
        self.right_motor.set(-1*(self.stick.getY())


if __name__ == "__main__":
    wpilib.run(MyRobot)