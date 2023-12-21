package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedStraightDriveCommand extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         time;
    private double         speed;
    private boolean        brakeAtEnd;



    private double         errorSignal;
    private double         previousError;
    private double         currentHeading;
    private double         targetHeading;
    private double         pTerm;
    private double         iTerm;
    private double         dTerm;


    public TimedStraightDriveCommand(double time, double speed, boolean brakeAtEnd, double targetHeading,
        DriveSubsystem driveSubsystem) {
        this.time           = time;
        this.speed          = speed;
        this.driveSubsystem = driveSubsystem;
        this.brakeAtEnd     = brakeAtEnd;
        this.targetHeading  = targetHeading;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        String commandParms = "time (ms): " + time + ", speed: " + speed + ", brake: "
            + brakeAtEnd + ", target heading (deg): " + targetHeading;

        logCommandStart(commandParms);

        currentHeading = driveSubsystem.getYaw();

    }

    @Override
    public void execute() {

        // executes every 20ms
        double currentError = driveSubsystem.getHeadingError(targetHeading);
        double diffError    = currentError - previousError;
        previousError  = currentError;

        pTerm          = DriveConstants.HEADING_PID_KP * currentError;
        iTerm         += DriveConstants.HEADING_PID_KI * currentError;
        dTerm         += DriveConstants.HEADING_PID_KD * currentError;


        errorSignal    = pTerm + iTerm + dTerm;

        double leftSpeed  = Math.min(Math.max(speed - errorSignal, -1.0), 1.0);
        double rightSpeed = Math.min(Math.max(speed + errorSignal, -1.0), 1.0);

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

    }

    @Override
    public boolean isFinished() {

        if (isTimeoutExceeded(time / 1000.0d)) {
            setFinishReason(time / 1000.0d + " seconds has been exceeded");
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
