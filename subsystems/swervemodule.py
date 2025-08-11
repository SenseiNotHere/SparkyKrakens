import math

from rev import SparkMax, SparkFlex, SparkLowLevel, SparkAbsoluteEncoder, SparkBase

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfigurator, TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import VelocityVoltage, PositionVoltage

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants, getSwerveDrivingMotorConfig, getSwerveTurningMotorConfig


class SwerveModule:
    def __init__(
            self,
            drivingCANId: int,
            drivingMotorType,
            turningCANId: int,
            turningMotorType,
            chassisAngularOffset: float,
            turnMotorInverted: bool,
    ) -> None:
        """
        Initializes a hybrid SwerveModule. TalonFX, SparkMax, and SparkFlex are
        supported motors for either driving or turning motors.
        :param drivingCANId:
        :param drivingMotorType:
        :param turningCANId:
        :param turningMotorType:
        :param chassisAngularOffset:
        :param turnMotorInverted:
        """
        
        if drivingMotorType not in (SparkMax, SparkFlex, TalonFX):
            raise ValueError("Invalid driving motor type. Must be SparkMax, SparkFlex, or TalonFX.")
        if turningMotorType not in (SparkMax, SparkFlex, TalonFX):
            raise ValueError("Invalid turning motor type. Must be SparkMax, SparkFlex, or TalonFX.")
        
        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d(0.0))
        
        # I'll start with SparkMax Driving
        if drivingMotorType is SparkMax:
            self.drivingMotor = SparkMax(
                drivingCANId,
                SparkLowLevel.MotorType.kBrushless
            )
            
            # Now we reset the motor to factory defaults
            self.drivingMotor.configure(
                getSwerveDrivingMotorConfig(),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
            
            self.drivingEncoder = self.drivingMotor.getEncoder()
            
            self.drivingPIDController = self.drivingMotor.getClosedLoopController()
            
            self.drivingEncoder.setPosition(0)

            self.drivingEncoder.setPositionConversionFactor(ModuleConstants.kDriveRotationsToMeters)
            self.drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDriveRPMToMps)

        # Now we can go to SparkFlex Driving
        elif drivingMotorType is SparkFlex:
            self.drivingMotor = SparkFlex(
                drivingCANId,
                SparkLowLevel.MotorType.kBrushless
            )

            # Now we reset the motor to factory defaults
            self.drivingMotor.configure(
                getSwerveDrivingMotorConfig(),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )

            self.drivingEncoder = self.drivingMotor.getEncoder()

            self.drivingPIDController = self.drivingMotor.getClosedLoopController()

            self.drivingEncoder.setPosition(0)

            self.drivingEncoder.setPositionConversionFactor(ModuleConstants.kDriveRotationsToMeters)
            self.drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDriveRPMToMps)


        # Now its time for TalonFX Driving
        elif drivingMotorType is TalonFX:
            self.drivingMotor = TalonFX(drivingCANId)
            
            self.drivingConfig = TalonFXConfiguration()
            self.drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
            
            # Setting appropriate PID Configurations
            self.drivingConfig.slot0.kP = ModuleConstants.kDrivingP
            self.drivingConfig.slot0.kI = ModuleConstants.kDrivingI
            self.drivingConfig.slot0.kD = ModuleConstants.kDrivingD
            self.drivingMotor.configurator.apply(self.drivingConfig)

            self.driving_velocity_request = VelocityVoltage(0).with_slot(0)

            self.resetEncoders()
        
        # Done with driving motors :P Going to turning motors now, SparkMax first
        if turningMotorType is SparkMax:
            self.turningMotor = SparkMax(
                turningCANId,

                SparkLowLevel.MotorType.kBrushless
            )
            
            # Now we reset the motor to factory defaults
            self.turningMotor.configure(
                getSwerveTurningMotorConfig(turnMotorInverted),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
            
            self.turningEncoder = self.turningMotor.getAbsoluteEncoder()
            
            self.turningEncoder.setPositionConversionFactor(2 * math.pi)  # rotations -> radians
            self.turningEncoder.setVelocityConversionFactor(2 * math.pi)  # rotations/s -> rad/s
            
            self.turningPIDController = self.turningMotor.getClosedLoopController()
            self.turningPIDController.setFeedbackDevice(self.turningEncoder)

            self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
            
        # We now go to SparkFlex Turning
        elif turningMotorType is SparkFlex:
            self.turningMotor = SparkFlex(
                turningCANId,
                SparkLowLevel.MotorType.kBrushless
            )

            # Now we reset the motor to factory defaults
            self.turningMotor.configure(
                getSwerveTurningMotorConfig(turnMotorInverted),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )

            self.turningEncoder = self.turningMotor.getAbsoluteEncoder()

            self.turningEncoder.setPositionConversionFactor(2 * math.pi)
            self.turningEncoder.setVelocityConversionFactor(2 * math.pi)

            self.turningPIDController = self.turningMotor.getClosedLoopController()
            self.turningPIDController.setFeedbackDevice(self.turningEncoder)

            self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        
        # Finally, we go to TalonFX Turning
        elif turningMotorType is TalonFX:
            self.turningMotor = TalonFX(turningCANId)
            
            turningConfig = TalonFXConfiguration()
            turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
            
            turningConfig.slot0.kP = ModuleConstants.kTurningP
            turningConfig.slot0.kI = ModuleConstants.kTurningI
            turningConfig.slot0.kD = ModuleConstants.kTurningD
            
            turningConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            self.turningMotor.configurator.apply(turningConfig)

            self.turning_position_request = PositionVoltage(0).with_slot(0)
            
            self.resetEncoders()
            
    def getState(self) -> SwerveModuleState:
        """
        Returns the current state of the swerve module.
        :return: SwerveModuleState
        """
        
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            self.drivingState = self.drivingEncoder.getVelocity()
        elif isinstance(self.drivingMotor, TalonFX):
            drive_rps = self.drivingMotor.get_velocity().value
            self.drivingState = (drive_rps / ModuleConstants.kDriveGearRatio) * ModuleConstants.kWheelCircumferenceMeters
        
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            self.turningState = self.turningEncoder.getPosition() - self.chassisAngularOffset
        elif isinstance(self.turningMotor, TalonFX):
            turn_motor_rot = self.turningMotor.get_position().value
            module_rot = turn_motor_rot / ModuleConstants.kTurnGearRatio
            self.turningState = (module_rot * 2 * math.pi) - self.chassisAngularOffset
        return SwerveModuleState(
            self.drivingState,
            Rotation2d(self.turningState)
        )
    
    def getPosition(self) -> SwerveModulePosition:
        """
        Returns the current position of the swerve module.
        :return: SwerveModulePosition
        """

        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            distance = self.drivingEncoder.getPosition()
        elif isinstance(self.drivingMotor, TalonFX):
            drive_motor_rot = self.drivingMotor.get_position().value
            wheel_rot = drive_motor_rot / ModuleConstants.kDriveGearRatio
            distance = wheel_rot * ModuleConstants.kWheelCircumferenceMeters
            
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            angle = self.turningEncoder.getPosition() - self.chassisAngularOffset
        elif isinstance(self.turningMotor, TalonFX):
            turn_motor_rot = self.turningMotor.get_position().value
            module_rot = turn_motor_rot / ModuleConstants.kTurnGearRatio
            angle = (module_rot * 2 * math.pi) - self.chassisAngularOffset
        return SwerveModulePosition(
            distance,
            Rotation2d(angle)
        )
    
    def setDesiredState(self,desiredState: SwerveModuleState) -> None:
        """
        Sets the desired state of the swerve module.
        :param desiredState:
        :return:
        """

        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            deg = desiredState.angle.degrees() % 90.0
            inXBrake = min(deg, 90.0 - deg) < 5.0  # 5° tol
            if not inXBrake:
                self.stop()
                return

        # Apply chassis angular offset to desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(self.chassisAngularOffset)
        
        # Optimize the reference state to avoid spinning further than 90 degrees.
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            turningPosition = self.turningEncoder.getPosition() - self.chassisAngularOffset  # radians
        elif isinstance(self.turningMotor, TalonFX):
            turn_motor_rot = self.turningMotor.get_position().value
            module_rot = turn_motor_rot / ModuleConstants.kTurnGearRatio
            turningPosition = (module_rot * 2 * math.pi) - self.chassisAngularOffset  # radians

        turningPosition = ((turningPosition + math.pi) % (2 * math.pi)) - math.pi

        optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d(turningPosition)
        )
        
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            self.drivingPIDController.setReference(optimizedDesiredState.speed, SparkLowLevel.ControlType.kVelocity)
        elif isinstance(self.drivingMotor, TalonFX):
            desired_drive_rps = (optimizedDesiredState.speed / ModuleConstants.kWheelCircumferenceMeters) \
                                * ModuleConstants.kDriveGearRatio
            self.drivingMotor.set_control(self.driving_velocity_request.with_velocity(desired_drive_rps))

        self.desiredState = optimizedDesiredState

        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            self.turningPIDController.setReference(
                optimizedDesiredState.angle.radians(), SparkLowLevel.ControlType.kPosition
            )
        elif isinstance(self.turningMotor, TalonFX):
            desired_module_rot = optimizedDesiredState.angle.radians() / (2 * math.pi)
            desired_motor_rot = desired_module_rot * ModuleConstants.kTurnGearRatio
            self.turningMotor.set_control(
                self.turning_position_request.with_position(desired_motor_rot)
            )

    def stop(self) -> None:
        # hold turn
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            hold = self.turningEncoder.getPosition()  # radians
            self.turningPIDController.setReference(hold, SparkLowLevel.ControlType.kPosition)
        elif isinstance(self.turningMotor, TalonFX):
            cur_rot = self.turningMotor.get_position().value
            self.turningMotor.set_control(self.turning_position_request.with_position(cur_rot))

        # zero drive
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            self.drivingPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
        elif isinstance(self.drivingMotor, TalonFX):
            self.drivingMotor.set_control(self.driving_velocity_request.with_velocity(0))

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(0, self.desiredState.angle)

    def resetEncoders(self) -> None:
        """
        Resets the encoders of the swerve module.
        :return:
        """
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            self.drivingEncoder.setPosition(0)
        elif isinstance(self.drivingMotor, TalonFX):
            self.drivingMotor.set_position(0)
