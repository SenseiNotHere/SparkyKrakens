# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""


import math

import rev
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from rev import SparkBase, SparkBaseConfig, ClosedLoopConfig
from phoenix6.hardware import talon_fx
from phoenix6.signals import NeutralModeValue
from pathplannerlib.config import RobotConfig, ModuleConfig, DCMotor


class NeoMotorConstants:
    kFreeSpeedRpm = 5676

class KrakenX60:
    kFreeSpeedRpm = 6000


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 1.0  # radians per second
    kMagnitudeSlewRate = 1.0  # percent per second (1 = 100%)
    kRotationalSlewRate = 1.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(26.5)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # set it to True if you were using a ruler for zeroing and want to ignore the offsets below
    kAssumeZeroOffsets = False

    # set the above to == False, if you are using Rev zeroing tool (and you have to tinker with offsets below)
    kFrontLeftChassisAngularOffset = units.degreesToRadians(180)
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = units.degreesToRadians(183)
    kBackRightChassisAngularOffset = units.degreesToRadians(-2)
    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 2
    kRearLeftDrivingCanId = 10
    kFrontRightDrivingCanId = 4
    kRearRightDrivingCanId = 6

    kFrontLeftTurningCanId = 3
    kRearLeftTurningCanId = 1
    kFrontRightTurningCanId = 5
    kRearRightTurningCanId = 7

    kGyroReversed = -1  # can be +1 if not flipped (affects field-relative driving)

class LiftConstants:
    kLeadLift = 8
    kFollowLift = 9

class IntakeConstants:
    kLeadIntake = 11
    kFollowIntake = 12
    kIntake = 13

def getSwerveDrivingMotorConfig() -> SparkBaseConfig:
    drivingConfig = SparkBaseConfig()
    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    drivingConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    return drivingConfig


def getSwerveTurningMotorConfig(turnMotorInverted: bool) -> SparkBaseConfig:
    turningConfig = SparkBaseConfig()
    turningConfig.inverted(turnMotorInverted)
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)
    turningConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    turningConfig.closedLoop.pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
    turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    turningConfig.closedLoop.positionWrappingEnabled(True)
    turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor)
    return turningConfig


def getSwerveDrivingMotorConfig() -> SparkBaseConfig:
    drivingConfig = SparkBaseConfig()
    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    drivingConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    return drivingConfig


def getSwerveTurningMotorConfig(turnMotorInverted: bool) -> SparkBaseConfig:
    turningConfig = SparkBaseConfig()
    turningConfig.inverted(turnMotorInverted)
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)
    turningConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    turningConfig.closedLoop.pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
    turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    turningConfig.closedLoop.positionWrappingEnabled(True)
    turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor)
    return turningConfig


class ModuleConstants:
    # WATCH OUT:
    #  - one or both of two constants below need to be flipped from True to False (by trial and error)
    #  , depending which swerve module you have (MK4i, MK4n, Rev, WCP, ThriftyBot, etc)
    kTurningEncoderInverted = False
    kTurningMotorInverted = True

    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 14

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.0624
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1  # can be dialed down if you see oscillations in the turning motor
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkBase.IdleMode.kBrake
    kTurningMotorIdleMode = SparkBase.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp

    kDrivingMinSpeedMetersPerSecond = 0.01


class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.02
    kOperatorControllerPort = 1
    kOperatorDeadband = 0.05

class DrivingConstants:
    """TalonFX-specific driving constants for Kraken X60 motors"""
    #Driving Motors
    kFrontLeftDriving = 1
    kFrontRightDriving = 2
    kBackLeftDriving = 3
    kBackRightDriving = 4

    #Turning Motors
    kFrontLeftTurning = 5
    kFrontRightTurning = 6
    kBackLeftTurning = 7
    kBackRightTurning = 8

    #Other settings
    #These are the maximum speeds the robot can reach, not maximum speeds allowed.
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    #Slew rate settings
    kDirectionSlewRate = 1.2  # radians per second
    kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(26.5)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    kAssumeZeroOffsets = False

    # set the above to == False, if you are manually zeroing (and you have to tinker with offsets below)
    kFrontLeftChassisAngularOffset = units.degreesToRadians(180)
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = units.degreesToRadians(183)
    kBackRightChassisAngularOffset = units.degreesToRadians(-2)

    kGyroReversed = -1

class TalonFXModuleConstants:
    """TalonFX-specific module constants for Kraken X60 motors"""
    #One needs to be false and the other true! Which is false and which is true depends on which swerve type you have.
    kTurningEncoderInverted = True
    kTurningMotorInverted = False

    kDrivingMotorPinionTeeth = 14

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1  # can be dialed down if you see oscillations in the turning motor
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.BRAKE)
    kTurningMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.COAST)

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp

    kDrivingMinSpeedMetersPerSecond = 0.01


class AutoConstants:
    """TalonFX-specific auto constants with PathPlanner configuration"""
    moduleConfig = ModuleConfig(
        maxDriveVelocityMPS=DrivingConstants.kMaxMetersPerSecond,
        driveMotor=DCMotor.krakenX60(),
        driveCurrentLimit=TalonFXModuleConstants.kDrivingMotorCurrentLimit,
        numMotors=4,
        wheelRadiusMeters=TalonFXModuleConstants.kWheelDiameterMeters / 2,
        wheelCOF=1.0
    )

    config = RobotConfig(
        massKG=60.00,
        MOI=8.0,
        moduleConfig=moduleConfig,
        moduleOffsets=DrivingConstants.kModulePositions
    )

    #Additional settings
    config.maxModuleSpeed = DrivingConstants.kMaxMetersPerSecond
    config.driveBaseRadius = 0.45  # meters
    config.maxCentripetalAcceleration = 3.0  # m/s²

    kUseSqrtControl = True  # improves arrival time and precision for simple driving commands

    # below are really trajectory constants
    kMaxMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )

    # below are really trajectory constants
    kMaxSpeedMetersPerSecond = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

