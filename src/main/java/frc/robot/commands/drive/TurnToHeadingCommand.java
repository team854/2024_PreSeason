package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeadingCommand extends BaseDriveCommand {

    private DriveSubsystem driveSubsystem;

    private double         timeoutTimeMS;

    private boolean        brakeAtEnd;
    private double         targetHeading;

    private boolean        reached;



    public TurnToHeadingCommand(double speed, double targetHeading, boolean brakeAtEnd, double timeoutTimeMS,
        DriveSubsystem driveSubsystem) {
        super(driveSubsystem);

        this.brakeAtEnd     = brakeAtEnd;
        this.targetHeading  = targetHeading;
        this.timeoutTimeMS  = timeoutTimeMS;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {

        String commandParms = "brake: " + brakeAtEnd + ", target heading: " + targetHeading
            + ", timeout time (ms): " + timeoutTimeMS;
        logCommandStart(commandParms);

        reached = false;

    }

    @Override
    public void execute() {

        reached = turnToHeading(targetHeading);

    }

    @Override
    public boolean isFinished() {

        if (reached) {
            setFinishReason("Within tolerance");
            return true;
        }

        if (isTimeoutExceeded(timeoutTimeMS / 1000d)) {
            setFinishReason("timeout of " + timeoutTimeMS / 1000d + " seconds");
            return true;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }

        logCommandEnd(interrupted);
    }

}
