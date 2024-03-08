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
        operatorInput.configureButtonBindings(driveSubsystem, armSubsystem);

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
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
        autoPatternChooser.addOption("Drive Forward PID Timed", AutoPattern.DRIVE_FORWARD_PID_TIMED);
        autoPatternChooser.addOption("Drive Forward PID Measured", AutoPattern.DRIVE_FORWARD_PID_MEASURED);
        autoPatternChooser.addOption("One shot from outside", AutoPattern.OUTSIDE_ONE_SHOT);
        autoPatternChooser.addOption("Two shots from outside", AutoPattern.OUTSIDE_TWO_SHOT);
        autoPatternChooser.addOption("Three shots from the speaker", AutoPattern.SPEAKER_THREE_SHOT);
        autoPatternChooser.addOption("Four shots from the speaker", AutoPattern.SPEAKER_FOUR_SHOT);
        autoPatternChooser.addOption("One shot from the amp", AutoPattern.AMP_ONE_SHOT);
        autoPatternChooser.addOption("Two shots from the amp", AutoPattern.AMP_TWO_SHOT);
        autoPatternChooser.addOption("One shot amp, one shot into speaker, from amo", AutoPattern.AMP_ONE_SHOT_ONE_AMP);
        autoPatternChooser.addOption("One shot into amp", AutoPattern.AMP_ONE_AMP);
        autoPatternChooser.addOption("Two shots into amp", AutoPattern.AMP_TWO_AMP);

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
            armSubsystem);
    }
}
