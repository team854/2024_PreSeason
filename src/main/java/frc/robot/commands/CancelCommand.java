package frc.robot.commands;

import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command is used to safely stop the robot in its current position, and to cancel any running
 * commands
 */
public class CancelCommand extends LoggingCommandBase {

    private final OperatorInput  operatorInput;
    private final DriveSubsystem driveSubsystem;

    /**
     * Cancel the commands running on all subsystems.
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot
     * from moving.
     */
    public CancelCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem) {

        this.operatorInput  = operatorInput;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        /*
         * The Cancel command is not interruptable and only ends when the cancel button is released.
         */
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {

        logCommandStart();

        driveSubsystem.stop();
    }

    @Override
    public void execute() {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {

        // Only end if the cancel button is released
        if (!operatorInput.isCancel()) {
            return true;
        }

        return false;
    }

}
