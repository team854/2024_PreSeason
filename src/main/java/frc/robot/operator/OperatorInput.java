package frc.robot.operator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.arm.AmpShootCommand;
import frc.robot.commands.arm.IntakeCommand;
import frc.robot.commands.arm.LongShotCommand;
import frc.robot.commands.arm.ManualShootCommand;
import frc.robot.commands.arm.PivotShootCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.climb.LowerBothClimbersCommand;
import frc.robot.commands.climb.RaiseBothClimbersCommand;
import frc.robot.commands.drive.TimedStraightDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;

/**
 * The Operator input class is used to map buttons to functions and functions to commands
 * <p>
 * This class extends SubsystemBase so that the periodic() routine is called each loop. The periodic
 * routine can be used to send debug information to the dashboard.
 */
public class OperatorInput extends SubsystemBase {

    public enum Stick {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    public final GameController driverController = new GameController(
        OperatorConstants.DRIVER_CONTROLLER_PORT,
        OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);

    /*
     * Map all functions to buttons.
     * A function should be a description of the robot behavior it is triggering.
     */

    // Cancel all commands when the driver presses the Xbox controller three lines (aka. start)
    // button
    public boolean isCancel() {
        return driverController.getStartButton();
    }

    // drive methods
    public boolean getBoost() {
        return driverController.getLeftBumper();
    }

    public double getLeftY() {
        return driverController.getLeftY();
    }

    public double getLeftX() {
        return driverController.getLeftX();
    }

    public double getRightY() {
        return driverController.getRightY();
    }

    public double getRightX() {
        return driverController.getRightX();
    }

    public double getDriverControllerAxis(Stick stick, Axis axis) {
        switch (stick) {
        case LEFT:
            switch (axis) {
            case X:
                return driverController.getLeftX();
            case Y:
                return driverController.getLeftY();
            }
            break;

        case RIGHT:
            switch (axis) {
            case X:
                return driverController.getRightX();
            case Y:
                return driverController.getRightY();
            }
            break;
        }
        return 0;
    }

    public double getSpeed(DriveMode driveMode) {
        return getDriverControllerAxis(Stick.LEFT, Axis.Y);
    }

    public double getTurn(DriveMode driveMode) {
        double turn = 0;
        switch (driveMode) {
        case SINGLE_STICK_ARCADE:
            turn = getDriverControllerAxis(Stick.LEFT, Axis.X);
            break;

        case DUAL_STICK_ARCADE:
        default:
            turn = getDriverControllerAxis(Stick.RIGHT, Axis.X);
            break;
        }
        return turn;
    }

    public double getLeftSpeed() {
        return getDriverControllerAxis(Stick.LEFT, Axis.Y);
    }

    public double getRightSpeed() {
        return getDriverControllerAxis(Stick.RIGHT, Axis.Y);
    }

    // arm methods
    public boolean isIntake() {
        return driverController.getLeftTriggerAxis() >= 0.5;
    }

    public boolean isShootFront() {
        return driverController.getRightTriggerAxis() >= 0.5;
    }

    public boolean isShootBack() {
        return driverController.getRightBumper();
    }

    public boolean isShoot() {
        return driverController.getYButton();
    }

    public boolean isAmpShot() {
        return driverController.getXButton();
    }

    public boolean isPivotUp() {
        return driverController.getPOV() == 0;
    }

    public boolean isPivotDown() {
        return driverController.getPOV() == 180;
    }

    // climber methods
    public boolean isLowerClimbers() {
        return driverController.getAButton();
    }

    public boolean isRaiseClimbers() {
        return driverController.getBButton();
    }

    public boolean isLongShot() {
        return driverController.getPOV() == 90; // Right D-Pad
    }

    /**
     * Use this method to define your robotFunction -> command mappings.
     * NOTE: all subsystems should be passed into this method.
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem,
        LightsSubsystem lightsSubsystem) { // Add LightsSubsystem

        new Trigger(() -> isCancel())
            .onTrue(new CancelCommand(this, driveSubsystem));

        new Trigger(() -> isIntake())
            .onTrue(new IntakeCommand(this, armSubsystem, lightsSubsystem, false)); // Pass
                                                                                    // LightsSubsystem
                                                                                    // to
                                                                                    // IntakeCommand

        new Trigger(() -> isShootBack())
            .onTrue(new PivotShootCommand(1, 115, 5000, armSubsystem));

        new Trigger(() -> isShootFront())
            .onTrue(new PivotShootCommand(1, 65, 5000, armSubsystem));

        new Trigger(() -> isLowerClimbers())
            .onTrue(new LowerBothClimbersCommand(climbSubsystem, this));

        new Trigger(() -> isRaiseClimbers())
            .onTrue(new RaiseBothClimbersCommand(climbSubsystem, this));

        new Trigger(() -> isAmpShot())
            .onTrue(new AmpShootCommand(0.4, 112, 5000, armSubsystem)
                .deadlineWith(new TimedStraightDriveCommand(10000, -0.1, true, driveSubsystem.getHeading(), driveSubsystem)));

        new Trigger(() -> isShoot())
            .onTrue(new ShootCommand(1, armSubsystem));

        new Trigger(() -> isPivotUp())
            .onTrue(new ManualShootCommand(1, 3000, armSubsystem, this));

        new Trigger(() -> isPivotDown())
            .onTrue(new ManualShootCommand(1, 3000, armSubsystem, this));

        new Trigger(() -> isLongShot())
            .onTrue(new LongShotCommand(1, 40, armSubsystem));
    }

    @Override
    public void periodic() {
        // Display any operator input values on the smart dashboard.
        SmartDashboard.putString("Driver Controller", driverController.toString());
    }
}
