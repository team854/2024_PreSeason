package frc.robot.operator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * The Operator input class is used to map buttons to functions and functions to commands
 * <p>
 * This class extends SubsystemBase so that the periodic() routine is called each loop. The periodic
 * routine can be used to send debug information to the dashboard
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
     *
     * A function should be a description of the robot behavior it is triggering.
     *
     * This separation of concerns allows for remapping of the robot functions to different
     * controller buttons without the need to change the command or the trigger. The mapping
     * from controller button to function is done in the following methods.
     */

    // Cancel all commands when the driver presses the XBox controller three lines (aka. start)
    // button
    public boolean isCancel() {
        return driverController.getStartButton();
    }

    public boolean getBoost() {
        return driverController.getRightBumper();
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

    /**
     * Use this method to define your robotFunction -> command mappings.
     *
     * NOTE: all subsystems should be passed into this method.
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem) {

        new Trigger(() -> isCancel())

            .onTrue(new CancelCommand(this, driveSubsystem));

    }

    @Override
    public void periodic() {

        // Display any operator input values on the smart dashboard.

        SmartDashboard.putString("Driver Controller", driverController.toString());
    }

}
