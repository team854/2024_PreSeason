package frc.robot.operatorInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorInput extends SubsystemBase {

    public final GameController driverController = new GameController(
        OiConstants.DRIVER_CONTROLLER_PORT,
        OiConstants.GAME_CONTROLLER_STICK_DEADBAND);

    // Cancel all commands when the driver presses the XBox controller three lines (aka. start) button
    public boolean isCancel() {
        return driverController.getStartButton();
    }

    /**
     * Use this method to define your robotFunction -> command mappings.
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem) {

        new Trigger(() -> isCancel())
            .onTrue(new CancelCommand(this, driveSubsystem));

    }

}
