#!/usr/bin/env python3
"""
    This is a simple implementation of the robotpy rev bindings.  If you want to see some PID, check out the other repository.
"""

import wpilib
import rev

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
            """
            This function is called upon program startup and
            should be used for any initialization code.
            """
            # Xbox Joystick Controller
            self.Xbox = wpilib.XboxController(0)
            # Rev through CAN(lift)
            self.left_lift = rev.CANSparkMax(10, rev.MotorType.kBrushless)
            self.right_lift = rev.CANSparkMax(11, rev.MotorType.kBrushless)
            #Timer
            self.timer = wpilib.Timer()
    def autonomousInit(self):
            """This function is run once each time the robot enters autonomous mode."""
            self.timer.reset()
            self.timer.start()
    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            self.left_lift.set(0.5)  # Drive forwards at half speed
        else:
            self.left_lift.set(0)  # Stop robot
    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.Xbox.set(getX(hand=<Hand.kRight: 1>))
        self.Xbox.set(getY(hand=<Hand.kLeft: 1>))


if __name__ == "__main__":
    wpilib.run(MyRobot)