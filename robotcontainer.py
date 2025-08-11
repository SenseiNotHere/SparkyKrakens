from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, PS4Controller, SmartDashboard
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from pathplannerlib.auto import AutoBuilder
from commands.holonomicdrive import HolonomicDrive
from buttonbindings import ButtonBindings
from constants import OIConstants
from autos.controller import ManAutoBuilder

class RobotContainer:
    """
    The container for the robot. Subsystems are initialized here, button bindings are set up, and
    auto chooser sent to the dashboard.
    """

    def __init__(self, robot):
        #The robot's subsystems
        self.robotDrive = DriveSubsystem()
        if wpilib.RobotBase.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)
            
        self.usePathPlanner = True

        if self.usePathPlanner:
            self.autoChooser = AutoBuilder.buildAutoChooser()
        else:
            self.manAutoBuilder = ManAutoBuilder(self.robotDrive)
            self.autoChooser = self.manAutoBuilder.configure_autos()

        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        #Setting up controllers
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)

        self.buttonBindings = ButtonBindings(self)
        self.buttonBindings.configureButtonBindings()

        self.robotDrive.setDefaultCommand(HolonomicDrive(
            self.robotDrive,
            forwardSpeed = lambda: -self.driverController.getRawAxis(PS4Controller.Axis.kLeftY),
            leftSpeed = lambda: -self.driverController.getRawAxis(PS4Controller.Axis.kLeftX),
            rotationSpeed = lambda: -self.driverController.getRawAxis(PS4Controller.Axis.kRightX),
            deadband = OIConstants.kDriveDeadband,
            fieldRelative = True,
            rateLimit = True,
            square = True,
        ))

    def disablePIDSubsystems(self):
        """
        Disables all PID Subsystems
        """

    def getAutonomousCommand(self):
        """
        Returns the command that will be run when autonomous mode is enabled.
        """
        selected = self.autoChooser.getSelected()
        if selected is None:
            print("WARNING: No autonomous selected!")
            return InstantCommand()
        try:
            print("Running autonomous routine:", selected.getName())
        except AttributeError:
            pass
        return selected

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command that will be running when test mode is enabled
        """