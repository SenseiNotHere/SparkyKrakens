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

            self.velocity_request = VelocityVoltage(0).with_slot(0)
            self.position_request = PositionVoltage(0).with_slot(0)

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
            
            self.turningPIDController = self.turningMotor.getClosedLoopController()
            
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

            self.turningPIDController = self.turningMotor.getClosedLoopController()

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
            
            self.velocity_request = VelocityVoltage(0).with_slot(0)
            self.position_request = PositionVoltage(0).with_slot(0)
            
            self.resetEncoders()
            
    def getState(self) -> SwerveModuleState:
        """
        Returns the current state of the swerve module.
        :return: SwerveModuleState
        """
        self.drivingState = None
        self.turningState = None
        
        # Driving motor values
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            self.drivingState = self.drivingEncoder.getVelocity()
        elif isinstance(self.drivingMotor, TalonFX):
            self.drivingState = self.drivingMotor.get_velocity().value
        
        # Turning motor values
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            self.turningState = self.turningEncoder.getPosition() - self.chassisAngularOffset
        elif isinstance(self.turningMotor, TalonFX):
            self.turningState = self.turningMotor.get_position().value - self.chassisAngularOffset
            
        return SwerveModuleState(
            self.drivingState,
            Rotation2d(self.turningState)
        )
    
    def getPosition(self) -> SwerveModulePosition:
        """
        Returns the current position of the swerve module.
        :return: SwerveModulePosition
        """
        
        distance = None
        angle = None
        
        # Driving motor values
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            distance = self.drivingEncoder.getPosition()
        elif isinstance(self.drivingMotor, TalonFX):
            distance = self.drivingMotor.get_position().value
            
        # Turning motor values
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            angle = self.turningEncoder.getPosition() - self.chassisAngularOffset
        elif isinstance(self.turningMotor, TalonFX):
            angle = self.turningMotor.get_position().value - self.chassisAngularOffset
            
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
            # If the speed is too low, we stop the module. Brownout prevention.
            inXBrake = abs(abs(desiredState.angle.degrees()) -  45) < 0.01
            
            if not inXBrake:
                self.stop()
                return
            
        # Apply chassis angular offset to desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(self.chassisAngularOffset)
        
        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = correctedDesiredState
        
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            optimizedDesiredState = correctedDesiredState
        elif isinstance(self.turningMotor, TalonFX):
            optimizedDesiredState = correctedDesiredState
        
        turningPosition = None
        
        if isinstance(self.turningMotor, (SparkMax, SparkFlex)):
            turningPosition = self.turningEncoder.getPosition()
        elif isinstance(self.turningMotor, TalonFX):
            turningPosition = self.turningMotor.get_position().value
            
        SwerveModuleState.optimize(optimizedDesiredState, Rotation2d(turningPosition))
        
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            self.drivingPIDController.setReference(optimizedDesiredState.speed, SparkLowLevel.ControlType.kVelocity)
            self.turningPIDController.setReference(optimizedDesiredState.angle.radians(), SparkLowLevel.ControlType.kPosition)
        elif isinstance(self.drivingMotor, TalonFX):
            self.velocity_request.value = optimizedDesiredState.speed
            self.position_request.value = optimizedDesiredState.angle.radians()

            self.drivingMotor.set(self.velocity_request.value)
            self.turningMotor.set(self.position_request.value)
            
        self.desiredState = optimizedDesiredState
            
    def stop(self) -> None:
        """
        Stops the swerve module.
        :return:
        """
        if isinstance(self.drivingMotor, (SparkMax, SparkFlex)):
            self.drivingPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
            self.turningPIDController.setReference(0, SparkLowLevel.ControlType.kPosition)
            
            if self.desiredState.speed is not 0:
                self.desiredState = SwerveModuleState(0, self.desiredState.angle)
                
        elif isinstance(self.drivingMotor, TalonFX):
            self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
            currentPositon = self.turningMotor.get_position().value
            self.turningMotor.set_control(self.position_request.with_position(currentPositon))

            if self.desiredState.speed is not 0:
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
