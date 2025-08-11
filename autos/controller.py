import wpilib

from commands2 import cmd, InstantCommand, RunCommand, WaitCommand, ParallelCommandGroup, ParallelDeadlineGroup
from commands2.button import CommandGenericHID
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

class ManAutoBuilder:
    def __init__(self, drive):
        """
        Reference to the drive subsystem for manual auto building.
        """
        self.drive = drive
        self.chooser = wpilib.SendableChooser()

    def configure_autos(self) -> wpilib.SendableChooser:
        """
        Returns the auto chooser with all the autos configured.
        """
        # Example autos — swap in your real commands
        do_nothing = InstantCommand()  # ends immediately

        drive_fwd_2s = RunCommand(
            lambda: self.drive.arcadeDrive(0.4, 0.0), self.drive
        ).withTimeout(2.0)

        stop = InstantCommand(lambda: self.drive.arcadeDrive(0.0, 0.0), self.drive)

        # Populate chooser
        self.chooser.setDefaultOption("Do Nothing", do_nothing)
        self.chooser.addOption("Drive Forward 2s", drive_fwd_2s)
        self.chooser.addOption("Stop", stop)

        return self.chooser