package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.HeadingStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SwivelOnLeftWheelCommand extends LoggingCommand {

    private DriveSubsystem driveSubsystem;



    // PID
    private double        currentError;
    private double        previousError;
    private double        diffError;
    private double        errorSignal;
    private double        pTerm;
    private double        iTerm = 0;
    private double        dTerm;

    // Time measure
    private double        initTime;
    private double        passedTime;
    private double        timeoutTimeMS;

    private HeadingStates headingState;

    // Logging
    private double        speed;
    private boolean       brakeAtEnd;
    private String        reason;
    private double        targetHeading;



    public SwivelOnLeftWheelCommand(double speed, double targetHeading, boolean brakeAtEnd, double timeoutTimeMS,
        DriveSubsystem driveSubsystem) {
        this.speed          = speed;
        this.brakeAtEnd     = brakeAtEnd;
        this.targetHeading  = targetHeading;
        this.timeoutTimeMS  = timeoutTimeMS;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", brake: " + brakeAtEnd + ", target heading: " + targetHeading
            + ", timeout time (ms): " + timeoutTimeMS;
        logCommandStart(commandParms);

        previousError = driveSubsystem.getHeadingError(targetHeading);


        initTime      = System.currentTimeMillis();

        if (Math.abs(previousError) > 10) {
            headingState = HeadingStates.FAR;
        }
        else {
            headingState = HeadingStates.CLOSE;
        }


    }

    @Override
    public void execute() {

        // executes every 20ms

        currentError  = driveSubsystem.getHeadingError(targetHeading);
        diffError     = currentError - previousError;
        previousError = currentError;

        double sgnError = Math.abs(currentError) / currentError;

        switch (headingState) {

        case FAR:
        default:

            driveSubsystem.setMotorSpeeds(0, -speed * sgnError);
            break;

        case CLOSE:

            pTerm = DriveConstants.TURN_TO_HEADING_PID_KP * currentError;
            iTerm += DriveConstants.TURN_TO_HEADING_PID_KI * currentError;
            dTerm += DriveConstants.TURN_TO_HEADING_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            Math.max(Math.min(errorSignal + Math.abs(errorSignal) / errorSignal * 0.2, 1), -1);

            driveSubsystem.setMotorSpeeds(0, -errorSignal * sgnError);

            break;
        }

        if (Math.abs(previousError) > 10) {
            headingState = HeadingStates.FAR;
        }
        else {
            headingState = HeadingStates.CLOSE;
        }


    }

    @Override
    public boolean isFinished() {

        // executes every 20ms

        currentError = driveSubsystem.getHeadingError(targetHeading);


        if (Math.abs(currentError) <= DriveConstants.HEADING_ERROR_BUFFER) {
            reason = "Within buffer accuracy";
            return true;
        }


        passedTime = System.currentTimeMillis();
        if (passedTime - initTime > timeoutTimeMS) {
            reason = "timeout";
            return true;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }


        setFinishReason(reason);
        logCommandEnd(interrupted);
    }

}
