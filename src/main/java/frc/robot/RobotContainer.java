// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The operator input class
    private final OperatorInput                operatorInput      = new OperatorInput();

    // The robot's subsystems and commands are defined here...
    private final LightsSubsystem              lightsSubsystem    = new LightsSubsystem();
    private final DriveSubsystem               driveSubsystem     = new DriveSubsystem();
    private final ArmSubsystem                 armSubsystem       = new ArmSubsystem();
    private final ClimbSubsystem               climbSubsystem     = new ClimbSubsystem();

    // All dashboard choosers are defined here...
    private final SendableChooser<DriveMode>   driveModeChooser   = new SendableChooser<>();
    private final SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Initialize the dashboard choosers
        initDashboardChoosers();

        // Initialize all Subsystem default commands.
        driveSubsystem.setDefaultCommand(
            new DefaultDriveCommand(operatorInput, driveModeChooser, driveSubsystem, lightsSubsystem));

        armSubsystem.setDefaultCommand(
            new DefaultArmCommand(operatorInput, armSubsystem));

        // Initialize the default command of the subsystem, to keep the arm hovering over the
        // ground.
        // armSubsystem.setDefaultCommand(new DefaultKeepArmUpCommand(armSubsystem));

        // Configure the button bindings
        operatorInput.configureButtonBindings(driveSubsystem, armSubsystem, climbSubsystem);

        // Add a trigger to flash the lights when the robot goes from disabled to enabled
        new Trigger(() -> RobotController.isSysActive())
            .onTrue(new InstantCommand(() -> lightsSubsystem.setEnabled()));
    }

    private void initDashboardChoosers() {

        driveModeChooser.setDefaultOption("Dual Stick Arcade", DriveMode.DUAL_STICK_ARCADE);
        SmartDashboard.putData("Drive Mode", driveModeChooser);
        driveModeChooser.addOption("Single Stick Arcade", DriveMode.SINGLE_STICK_ARCADE);
        driveModeChooser.addOption("Tank", DriveMode.TANK);

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
        autoPatternChooser.addOption("Blue and red Speaker side 2 shot", AutoPattern.BLUE_RED_SPEAKER_TWO_SHOT);

        // autoPatternChooser.addOption("Red Outside side 2 shot",
        // AutoPattern.RED_OUTSIDE_TWO_SHOT);
        // autoPatternChooser.addOption("Red Speaker side 3 shot",
        // AutoPattern.RED_SPEAKER_THREE_SHOT);
        // autoPatternChooser.addOption("Blue Speaker side 3 shot",
        // AutoPattern.BLUE_SPEAKER_THREE_SHOT);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Pass in all of the subsystems and all of the choosers to the auto command.
        return new AutonomousCommand(
            driveSubsystem,
            autoPatternChooser,
            armSubsystem,
            operatorInput);
    }
}
