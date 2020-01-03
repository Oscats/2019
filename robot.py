#!/usr/bin/env python3
# THIS IS THE ULTIMATE MASTER CODE
# DRIVE BASE
# ELEVATOR
# INTAKE
# PNEUMATICS


import wpilib
import ctre
import wpilib.drive
import rev
from wpilib.interfaces import GenericHID
Hand = GenericHID.Hand

from networktables import NetworkTables


class MyRobot(wpilib.TimedRobot):
    #Set up PID COnstants...
    if wpilib.RobotBase.isSimulation():
        # These PID parameters are used in simulation
        kP = 0.03
        kI = 0.00
        kD = 0.00
        kF = 0.00
    else:
        # These PID parameters are used on a real robot
        kP = 0.045
        kI = 0.00
        kD = 0.0
        kF = 0.00

    kToleranceDegrees = 2.0
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
        if wpilib.RobotBase.isSimulation():
            self.climber = rev.CANSparkMax(12, rev.MotorType.kBrushless)
        else: 
            self.eleRight.follow(self.eleLeft, invert = True)

        # Thor's Stabilizer
        #self.StaLeft = rev.CANSparkMax(12, rev.MotorType.kBrushless)
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
        self.sd.putNumber('Drive Limit', .5)
        


        #Get items from Shuffeboard
        self.driveLimit = self.sd.getNumber('Drive Limit',.5)
        #Construct the Pigeon
        self.pigeon = ctre.pigeonimu.PigeonIMU(self.frontleft_motor)
        self.yaw = lambda: self.pigeon.getYawPitchRoll()[0]
        #Construct communication with the pi and the Hephestus output
        self.hephestus=NetworkTables.getTable('hephestus')

        #Consruct the line tracker inputs
        self.leftLine=wpilib.AnalogInput(0)
        self.centerLeftLine=wpilib.AnalogInput(1)
        self.centerRightLine=wpilib.AnalogInput(2)
        self.lineRightLine=wpilib.AnalogInput(3)

        #Construct the trun controller...
        turnController = wpilib.PIDController(
        self.kP, self.kI, self.kD, self.kF,self.yaw , output=self
        )
        
        turnController.setInputRange(-180.0, 180.0)
        turnController.setOutputRange(-.5, .5)
        turnController.setAbsoluteTolerance(self.kToleranceDegrees)
        turnController.setContinuous(True)

        self.turnController = turnController
        self.rotateToAngleRate = 0



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
        # drive code
        # Begin by attempting to grab the offset for Hephestus (how far off the target is the robot facing?)
        try:
            rawTargetOffset = self.hephestus.getNumber('targetOffset',0)
            #Convert the offset to a useable motor range using a proportion.
            OldRange = (156 - (-156))
            NewRange = (1 - (-1))
            targetOffset = ((rawTargetOffset/OldRange) * NewRange)
            #If we press button 5, turn to the target.
            if self.stick.getRawButton(5) == True:
                if targetOffset != 0:
                    turn = .8 * targetOffset
                    self.sd.putNumber('turn',turn)
                else:
                    turn = 0
            
            else:
                #Otherwise drive normally.
                turn = (self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1)+self.driveLimit)*(self.stick.getX(Hand.kRight))
        except:
            # If the above fails, drive normally.
            turn = (self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1)+self.driveLimit)*(self.stick.getX(Hand.kRight))
            self.sd.putNumber('turn',turn)
        # Attempt to get the line tracker info...
        try:
            leftLine = self.leftLine.getAverageVoltage()
            centerlLine = self.centerLeftLine.getAverageVoltage()
            centerRLine = self.rightCenterRightLine.getAverageVoltage()
            rightLine = self.RightLine.getAverageVoltage()
            self.sd.putNumberArray('Line Tracker', [leftLine, centerlLine, centerRLine, rightLine])
            if self.stick.getRawButton(6) == True:
                if leftLine >= .8:
                    turn = -0.6
                elif centerlLine >= .7 or centerRLine >= .7 :
                    turn = 0
                    straight = 0
                    if self.yaw() < straight:
                        turn = 0.03 #* abs(straight - self.yaw() - 0)
                    else:
                        turn = 0.0 #* abs(straight + self.yaw() - 0)                
                elif rightLine >= .8:
                    turn = 0.6
            else:
                turn = .8*self.stick.getX()
        except:
             # If the above fails, drive normally.
            turn = (self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1)+self.driveLimit)*(self.stick.getX(Hand.kRight))
            self.sd.putNumber('turn',turn)

        #Try to implement the turn controller.
        try:
            rotateToAngle = False

            #if self.stick.getRawButton(1):
            #    self.pigeon.reset()

            if self.stick.getRawButton(1):
                self.turnController.setSetpoint(0.0)
                rotateToAngle = True
            elif self.stick.getRawButton(2):
                self.turnController.setSetpoint(90.0)
                rotateToAngle = True
            elif self.stick.getRawButton(3):
                self.turnController.setSetpoint(179.9)
                rotateToAngle = True
            elif self.stick.getRawButton(4):
                self.turnController.setSetpoint(-90.0)
                rotateToAngle = True


            if rotateToAngle:
                self.turnController.enable()
                turn = self.rotateToAngleRate
            else:
                self.turnController.disable()
                turn = (self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1)+self.driveLimit)*(self.stick.getX(Hand.kRight))
        except:
            turn = (self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1)+self.driveLimit)*(self.stick.getX(Hand.kRight))
            self.sd.putNumber('turn',turn)

        self.drive.arcadeDrive(((self.stick.getTriggerAxis(Hand.kRight)*((1-self.driveLimit)/1))+self.driveLimit)*-1*(self.stick.getY(Hand.kRight)), turn) 
       
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
        self.left_motor.set((self.stick.getTriggerAxis(Hand.kLeft)*((1-.3)/1)+.3)*(self.stick.getY(Hand.kLeft)))
        self.right_motor.set((self.stick.getTriggerAxis(Hand.kLeft)*((1-.3)/1)+.3)*(self.stick.getY(Hand.kLeft)))
        
        # elevator
        self.eleLeft.set((self.stick2.getY(Hand.kLeft)))
        #self.eleRight.set((self.stick2.getY(Hand.kLeft)))
        #
        self.intake_angle.set(self.stick2.getY(Hand.kRight))
        #Thor  <--- We did not implement Thor this season...
        #self.StaLeft.set(0+(self.stick2.getTriggerAxis(Hand.kLeft))-(self.stick2.getTriggerAxis(Hand.kRight)))

        # note: Xbox controller 1 controlls the drive base and stablizing/rising pneu. and intake
        # note: Xbox controller 2 controlls the elevator and hatchcover thing
    
    def pidWrite(self, output):
        """This function is invoked periodically by the PID Controller,
        based upon Pigeon yaw angle input and PID Coefficients.
        """
        self.rotateToAngleRate = output

if __name__ == "__main__":
    wpilib.run(MyRobot)

   
   
