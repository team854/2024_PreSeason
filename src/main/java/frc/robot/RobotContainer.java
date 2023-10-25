// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.operator.GameController;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem               driveSubsystem     = new DriveSubsystem();

    // Drive mode chooser
    private final SendableChooser<DriveMode>   driveModeChooser   = new SendableChooser<>();

    // A set of choosers for autonomous patterns
    private final SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();

    // The driver's controller
    private final GameController               driverController   = new GameController(
        OiConstants.DRIVER_CONTROLLER_PORT, OiConstants.GAME_CONTROLLER_STICK_DEADBAND);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Initialize all Subsystem default commands.
        driveSubsystem
            .setDefaultCommand(new DefaultDriveCommand(driverController, driveSubsystem, driveModeChooser));

        // Initialize the autonomous choosers
        initAutoSelectors();

        // Configure the button bindings
        configureButtonBindings();
    }

    private void initAutoSelectors() {

        driveModeChooser.setDefaultOption("Tank", DriveMode.TANK);
        SmartDashboard.putData("Drive Mode", driveModeChooser);
        driveModeChooser.addOption("Dual Stick Arcade", DriveMode.DUAL_STICK_ARCADE);
        driveModeChooser.addOption("Single Stick Arcade", DriveMode.SINGLE_STICK_ARCADE);


        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // FIXME pass in all of the choosers to an appropriate auto command.
        return new AutonomousCommand(
            driveSubsystem,
            autoPatternChooser);
    }
}
