package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;

public class SwivelOnLeftWheelCommand extends BaseDriveCommand {

    private DriveSubsystem driveSubsystem;

    private double         timeoutTimeMS;

    // Logging
    private double         speed;
    private boolean        brakeAtEnd;
    private double         targetHeading;

    private boolean        reached;



    public SwivelOnLeftWheelCommand(double speed, double targetHeading, boolean brakeAtEnd, double timeoutTimeMS,
        DriveSubsystem driveSubsystem) {
        super(driveSubsystem);
        this.speed         = speed;
        this.brakeAtEnd    = brakeAtEnd;
        this.targetHeading = targetHeading;
        this.timeoutTimeMS = timeoutTimeMS;
    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", brake: " + brakeAtEnd + ", target heading: " + targetHeading
            + ", timeout time (ms): " + timeoutTimeMS;
        logCommandStart(commandParms);

        super.initialize();

        reached = false;


    }

    @Override
    public void execute() {

        // executes every 20ms

        reached = swivelAboutLeft(speed, targetHeading);

    }

    @Override
    public boolean isFinished() {

        // executes every 20ms

        if (reached) {
            setFinishReason("Within tolerance");
            return true;
        }

        if (isTimeoutExceeded(timeoutTimeMS)) {
            setFinishReason("Within tolerance");
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
