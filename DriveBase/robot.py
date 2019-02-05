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

        # Left motor 
        self.frontleft_motor = wpilib.Victor(0)
        self.backleft_motor = wpilib.Victor(1) 
        self.leftdrive = wpilib.SpeedControllerGroup(self.frontleft_motor, self.backleft_motor)
        # Right motor
        self.frontright_motor = wpilib.Victor(2)
        self.backright_motor = wpilib.Victor(3)
        self.rightdrive = wpilib.SpeedControllerGroup(self.frontright_motor, self.backright_motor)
        # Drive - combining left + right
        self.drive = wpilib.drive.DifferentialDrive(self.leftdrive, self.rightdrive)


        self.stick = wpilib.Joystick(0)
        self.timer = wpilib.Timer()
        # wpilib.CameraServer.launch()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        self.drive.arcadeDrive(0, 0)  # Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
       
        
        self.drive.arcadeDrive(.5*(self.stick.getY()), (.5*(self.stick.getX())))


if __name__ == "__main__":
    wpilib.run(MyRobot)