package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.DriveConstants.RookieSettings;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class RobotContainer {

    // The operator input class
    private final OperatorInput                   operatorInput      = new OperatorInput();

    // The robot's subsystems and commands are defined here
    private final LightsSubsystem                 lightsSubsystem    = new LightsSubsystem();            // LightsSubsystem
                                                                                                         // initialized
    private final DriveSubsystem                  driveSubsystem     = new DriveSubsystem();
    private final ArmSubsystem                    armSubsystem       = new ArmSubsystem();
    private final ClimbSubsystem                  climbSubsystem     = new ClimbSubsystem(operatorInput);

    // All dashboard choosers are defined here
    private final SendableChooser<DriveMode>      driveModeChooser   = new SendableChooser<>();
    private final SendableChooser<AutoPattern>    autoPatternChooser = new SendableChooser<>();
    private final SendableChooser<RookieSettings> speedChooser       = new SendableChooser<>();

    public RobotContainer() {

        // Initialize the dashboard choosers
        initDashboardChoosers();

        // Set default commands for subsystems
        driveSubsystem.setDefaultCommand(
            new DefaultDriveCommand(operatorInput, speedChooser, driveModeChooser, driveSubsystem, lightsSubsystem));

        armSubsystem.setDefaultCommand(
            new DefaultArmCommand(operatorInput, armSubsystem));

        // Configure the button bindings (ensure LightsSubsystem is passed)
        operatorInput.configureButtonBindings(driveSubsystem, armSubsystem, climbSubsystem, lightsSubsystem);

        // Add a trigger to flash the lights when the robot goes from disabled to enabled
        new Trigger(() -> RobotController.isSysActive())
            .onTrue(new InstantCommand(() -> lightsSubsystem.setEnabled()));
    }

    private void initDashboardChoosers() {

        // Drive Mode chooser
        driveModeChooser.setDefaultOption("Dual Stick Arcade", DriveMode.DUAL_STICK_ARCADE);
        SmartDashboard.putData("Drive Mode", driveModeChooser);
        driveModeChooser.addOption("Single Stick Arcade", DriveMode.SINGLE_STICK_ARCADE);
        driveModeChooser.addOption("Tank", DriveMode.TANK);

        // Auto Pattern chooser
        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Test Arm Commands", AutoPattern.TEST_ARM_COMMANDS);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
        autoPatternChooser.addOption("Drive Forward PID Timed", AutoPattern.DRIVE_FORWARD_PID_TIMED);
        autoPatternChooser.addOption("Drive Forward PID Measured", AutoPattern.DRIVE_FORWARD_PID_MEASURED);

        autoPatternChooser.addOption("Blue Outside side 1 shot", AutoPattern.BLUE_OUTSIDE_ONE_SHOT);
        autoPatternChooser.addOption("Red Outside side 1 shot", AutoPattern.RED_OUTSIDE_ONE_SHOT);

        autoPatternChooser.addOption("Blue Amp side 1 shot", AutoPattern.BLUE_AMP_ONE_SHOT);
        autoPatternChooser.addOption("Blue Amp side 2 shot", AutoPattern.BLUE_AMP_TWO_SHOT);
        autoPatternChooser.addOption("Red Amp side 1 shot", AutoPattern.RED_AMP_ONE_SHOT);
        autoPatternChooser.addOption("Red Amp side 2 shot", AutoPattern.RED_AMP_TWO_SHOT);

        autoPatternChooser.addOption("Red Speaker side 1 shot", AutoPattern.RED_SPEAKER_ONE_SHOT);
        autoPatternChooser.addOption("Red Speaker side 2 shot", AutoPattern.RED_SPEAKER_TWO_SHOT);
        autoPatternChooser.addOption("Blue Speaker side 1 shot", AutoPattern.BLUE_SPEAKER_ONE_SHOT);
        autoPatternChooser.addOption("Blue Speaker side 2 shot", AutoPattern.BLUE_SPEAKER_TWO_SHOT);

        // Speed chooser (Rookie vs Normal mode)
        speedChooser.setDefaultOption("Normal driver", RookieSettings.NORMAL);
        speedChooser.addOption("Rookie driver", RookieSettings.ROOKIE);
        SmartDashboard.putData("Rookie Mode", speedChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Pass subsystems, choosers, and LightsSubsystem to AutonomousCommand
        return new AutonomousCommand(
            driveSubsystem,
            autoPatternChooser,
            armSubsystem,
            operatorInput,
            lightsSubsystem // Pass LightsSubsystem to AutonomousCommand
        );
    }
}
