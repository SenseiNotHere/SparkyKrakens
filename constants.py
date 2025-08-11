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
    # Driving Parameters - allowed max speeds
    kMaxSpeedMetersPerSecond = 3.0  # match autos & teleop for consistency
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 1.0  # radians per second
    kMagnitudeSlewRate = 1.0  # percent per second
    kRotationalSlewRate = 1.0  # percent per second

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    kAssumeZeroOffsets = False

    # Module angular offsets
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

    kGyroReversed = -1


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


class ModuleConstants:
    # Conversion scalars (kept but match derived values below)
    kDriveRPMToMps = 0.00078805
    kDriveRotationsToMeters = 0.047283

    kTurningEncoderInverted = False
    kTurningMotorInverted = True

    # Wheel + gearing
    kDrivingMotorPinionTeeth = 14
    kWheelDiameterMeters = 0.0762  # 3" wheel; adjust if different
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0

    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0

    kTurningEncoderPositionPIDMinInput = 0
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkBase.IdleMode.kBrake
    kTurningMotorIdleMode = SparkBase.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50
    kTurningMotorCurrentLimit = 20

    kDrivingMinSpeedMetersPerSecond = 0.01

    kDriveGearRatio = 6.75
    kTurnGearRatio = 150.0


class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.02
    kOperatorControllerPort = 1
    kOperatorDeadband = 0.05


class DrivingConstants:
    # Driving Motors
    kFrontLeftDriving = 1
    kFrontRightDriving = 2
    kBackLeftDriving = 3
    kBackRightDriving = 4

    # Turning Motors
    kFrontLeftTurning = 5
    kFrontRightTurning = 6
    kBackLeftTurning = 7
    kBackRightTurning = 8

    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    kDirectionSlewRate = 1.2
    kMagnitudeSlewRate = 1.8
    kRotationalSlewRate = 2.0

    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    kAssumeZeroOffsets = False
    kFrontLeftChassisAngularOffset = units.degreesToRadians(180)
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = units.degreesToRadians(183)
    kBackRightChassisAngularOffset = units.degreesToRadians(-2)

    kGyroReversed = -1


class TalonFXModuleConstants:
    kTurningEncoderInverted = True
    kTurningMotorInverted = False

    kDrivingMotorPinionTeeth = 14
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0

    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0

    kTurningEncoderPositionPIDMinInput = 0
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.BRAKE)
    kTurningMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.COAST)

    kDrivingMotorCurrentLimit = 50
    kTurningMotorCurrentLimit = 20

    kDrivingMinSpeedMetersPerSecond = 0.01


class AutoConstants:
    moduleConfig = ModuleConfig(
        maxDriveVelocityMPS=DriveConstants.kMaxSpeedMetersPerSecond,
        driveMotor=DCMotor.krakenX60(),
        driveCurrentLimit=TalonFXModuleConstants.kDrivingMotorCurrentLimit,
        numMotors=4,
        wheelRadiusMeters=ModuleConstants.kWheelDiameterMeters / 2,
        wheelCOF=1.0
    )

    config = RobotConfig(
        massKG=60.00,
        MOI=8.0,
        moduleConfig=moduleConfig,
        moduleOffsets=DriveConstants.kModulePositions
    )

    config.maxModuleSpeed = DriveConstants.kMaxSpeedMetersPerSecond
    config.driveBaseRadius = 0.45
    config.maxCentripetalAcceleration = 3.0

    kUseSqrtControl = True

    kMaxMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )

    kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi