#!/usr/bin/env python3
# THIS IS THE ULTIMATE MASTER CODE
# DRIVE BASE
# ELEVATOR
# INTAKE
# PNEUMATICS
"""
    This is a good foundation to build your robot code on    
    This is a simple implementation of the robotpy rev bindings.  If you want to see some PID, check out the other repository.

"""

import wpilib
import ctre
import wpilib.drive
import rev
from wpilib.interfaces import GenericHID
Hand = GenericHID.Hand

from networktables import NetworkTables


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        # Left motor 
        self.frontleft_motor = ctre.WPI_TalonSRX(0)
        self.backleft_motor = ctre.WPI_TalonSRX(1) 
        self.leftdrive = wpilib.SpeedControllerGroup(self.frontleft_motor, self.backleft_motor)
        # Right motor
        self.frontright_motor = ctre.WPI_TalonSRX(2)
        self.backright_motor = ctre.WPI_TalonSRX(3)
        self.rightdrive = wpilib.SpeedControllerGroup(self.frontright_motor, self.backright_motor)
        # Drive - combining left + right
        self.drive = wpilib.drive.DifferentialDrive(self.leftdrive, self.rightdrive)

         # Elevator Rev through CAN(lift)+
        self.eleLeft = rev.CANSparkMax(10, rev.MotorType.kBrushless)
        self.eleRight = rev.CANSparkMax(11, rev.MotorType.kBrushless)
        #self.climber = rev.CANSparkMax(12, rev.MotorType.kBrushless)
        # self.eleRight.Follow(self.eleLeft)

        # Thor's Stabilizer
        self.StaLeft = rev.CANSparkMax(12, rev.MotorType.kBrushless)
        #self.StaRight = rev.CANSparkMax(14, rev.MotorType.kBrushless)
        #self.Lift = rev.CANSparkMax(152, rev.MotorType.kBrushless)
    
        # intake motors
        self.left_motor = ctre.WPI_VictorSPX(6)
        self.right_motor = ctre.WPI_VictorSPX(7)
        # intake angle
        self.intake_angle = ctre.WPI_TalonSRX(5)
       
        # intake stick & timer
         # elevator timer
        self.timer = wpilib.Timer()
         # pneumatics joystick
        self.stick = wpilib.XboxController(0)
        #construct Shuffleboard
        self.sd = NetworkTables.getTable('SmartDashboard')        
        # pneumatics solenoids
        self.hatchcover = wpilib.DoubleSolenoid(0,1)
        self.doubleSolenoid = wpilib.DoubleSolenoid(2,3)
        self.stick2 = wpilib.XboxController(1)
        
        # Construct Camera
        wpilib.CameraServer.launch()

        #construct Shuffleboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        #Put items on Shuffleboard
        
        self.sd.putNumber('liftLimit', .4)
        self.sd.putBoolean('LiftLimit', 0)
        


        #Get items from Shuffeboard
        self.driveLimit = self.sd.getNumber('driveLimit',.5)

        # Rip auto
        self.autonomousInit = self.teleopInit
        self.autonomousPeriodic = self.teleopPeriodic

    # def autonomousInit(self):
    #     """This function is run once each time the robot enters autonomous mode."""
    #     self.timer.reset()
    #     self.timer.start()

    # def autonomousPeriodic(self):
    #     """This function is called periodically during autonomous."""
    #    # intake printauto
    #    # print ("auto")
    #     # pneu Drive for two seconds
    #     if self.timer.get() < 2.0:
    #         self.drive.arcadeDrive(-0.5, 0)  # Drive forwards at half speed
    #     else:
    #         self.drive.arcadeDrive(0, 0)  # Stop robot
    
    def teleopInit(self):
        self.driveLimit = self.sd.getNumber('driveLimit',.5)
        

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        
        # Pneumatics
        if (self.stick2.getRawButton(2) == (1)):
            self.hatchcover.set(1)
            self.sd.putBoolean('Hatch?', True)

        elif (self.stick2.getRawButton(3)):
            self.hatchcover.set(2)
        else:
             self.hatchcover.set(0)
        if (self.stick.getRawButton(4) == (1)):
            self.doubleSolenoid.set(1)
        elif (self.stick.getRawButton(1)):
            self.doubleSolenoid.set(2)
        else:
             self.doubleSolenoid.set(0)

        # intake motor command
        self.left_motor.set((self.stick.getY(Hand.kLeft)))
        self.right_motor.set((self.stick.getY(Hand.kLeft)))

        # drive motors 
        self.drive.arcadeDrive(((self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1))+self.driveLimit)*-1*(self.stick.getY(Hand.kRight)), (self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1)+self.driveLimit)*(self.stick.getX(Hand.kRight)))
        
        # elevator
        self.eleLeft.set((self.stick2.getY(Hand.kLeft)))
        self.eleRight.set(-1*(self.stick2.getY(Hand.kLeft)))
        #
        self.intake_angle.set(self.stick2.getY(Hand.kRight))
        #Thor 
        self.StaLeft.set(0+(self.stick2.getTriggerAxis(Hand.kLeft))-(self.stick2.getTriggerAxis(Hand.kRight)))

        # note: Xbox controller 1 controlls the drive base and stablizing/rising pneu. and intake
        # note: Xbox controller 2 controlls the elevator and hatchcover thing
    

if __name__ == "__main__":
    wpilib.run(MyRobot)

   