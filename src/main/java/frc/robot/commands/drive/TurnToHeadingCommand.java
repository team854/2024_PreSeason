package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeadingCommand extends LoggingCommand {

    private DriveSubsystem driveSubsystem;
    private double         speed;
    private boolean        brakeAtEnd;


    private double         targetHeading;
    private double         currentError;
    private double         previousError;
    private double         diffError;
    private double         errorSignal;
    private double         pTerm;
    private double         iTerm = 0;
    private double         dTerm;

    private double         initTime;
    private double         passedTime;
    private double         timeoutTime;


    public TurnToHeadingCommand(double speed, double targetHeading, boolean brakeAtEnd, double timeoutTime,
        DriveSubsystem driveSubsystem) {
        this.speed          = speed;
        this.brakeAtEnd     = brakeAtEnd;
        this.targetHeading  = targetHeading;
        this.timeoutTime    = timeoutTime;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", brake: " + brakeAtEnd + ", target heading: " + targetHeading
            + ", timeout time (ms): " + timeoutTime;
        logCommandStart(commandParms);

        previousError = driveSubsystem.getHeadingError(targetHeading);

        initTime      = System.currentTimeMillis();


    }

    @Override
    public void execute() {

        // executes every 20ms

        currentError  = driveSubsystem.getHeadingError(targetHeading);
        diffError     = currentError - previousError;
        previousError = currentError;
        System.out.println(currentError);

        pTerm        = DriveConstants.TURN_TO_HEADING_PID_KP * currentError;
        iTerm       += DriveConstants.TURN_TO_HEADING_PID_KI * currentError;
        dTerm       += DriveConstants.TURN_TO_HEADING_PID_KD * diffError;

        errorSignal  = pTerm + iTerm + dTerm;

        Math.max(Math.min(errorSignal + Math.abs(errorSignal) / errorSignal * 0.2, 1), -1);

        driveSubsystem.setMotorSpeeds(speed, -speed);

    }

    @Override
    public boolean isFinished() {

        // executes every 20ms

        currentError = driveSubsystem.getHeadingError(targetHeading);


        if (Math.abs(currentError) <= DriveConstants.HEADING_ERROR_BUFFER) {
            return true;
        }


        passedTime = System.currentTimeMillis();
        if (passedTime - initTime > timeoutTime) {
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
