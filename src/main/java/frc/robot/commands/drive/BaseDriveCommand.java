package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.HeadingStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public abstract class BaseDriveCommand extends LoggingCommand {

    final DriveSubsystem driveSubsystem;

    double               previousError;

    double               pTerm;
    double               iTerm = 0;
    double               dTerm;
    double               errorSignal;

    public BaseDriveCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        previousError = 0;
        iTerm         = 0;
    }

    public void driveStraight(double speed) {
        double targetHeading = driveSubsystem.getHeading();

        double error         = driveSubsystem.getHeadingError(targetHeading);
        double diffError     = error - previousError;
        previousError  = error;

        pTerm          = DriveConstants.HEADING_PID_KP * error;
        iTerm         += DriveConstants.HEADING_PID_KI * error;
        dTerm          = DriveConstants.HEADING_PID_KD * diffError;


        errorSignal    = pTerm + iTerm + dTerm;

        double leftSpeed  = Math.min(Math.max(speed - errorSignal, -1.0), 1.0);
        double rightSpeed = Math.min(Math.max(speed + errorSignal, -1.0), 1.0);

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    public void driveOnHeading(double speed, double targetHeading) {

        double error     = driveSubsystem.getHeadingError(targetHeading);
        double diffError = error - previousError;
        previousError  = error;

        pTerm          = DriveConstants.HEADING_PID_KP * error;
        iTerm         += DriveConstants.HEADING_PID_KI * error;
        dTerm          = DriveConstants.HEADING_PID_KD * diffError;


        errorSignal    = pTerm + iTerm + dTerm;

        double leftSpeed  = Math.min(Math.max(speed - errorSignal, -1.0), 1.0);
        double rightSpeed = Math.min(Math.max(speed + errorSignal, -1.0), 1.0);

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    public boolean turnToHeading(double targetHeading) {

        HeadingStates headingState;

        if (Math.abs(previousError) > DriveConstants.TURN_TO_HEADING_CLOSE) {
            headingState = HeadingStates.FAR;
        }
        else {
            headingState = HeadingStates.CLOSE;
        }

        double error     = driveSubsystem.getHeadingError(targetHeading);
        double diffError = error - previousError;
        previousError = error;

        switch (headingState) {

        case FAR:
        default:
            driveSubsystem.setMotorSpeeds(DriveConstants.DEFAULT_TURN_SPEED * Math.signum(error),
                -DriveConstants.DEFAULT_TURN_SPEED * Math.signum(error));
            break;

        case CLOSE:

            pTerm = DriveConstants.TURN_TO_HEADING_PID_KP * error;
            iTerm += DriveConstants.TURN_TO_HEADING_PID_KI * error;
            dTerm = DriveConstants.TURN_TO_HEADING_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            Math.max(Math.min(errorSignal, 1), -1);

            driveSubsystem.setMotorSpeeds(-errorSignal, errorSignal);

            break;
        }

        if (error <= DriveConstants.TURN_TO_HEADING_TOLERANCE) {
            return true;
        }
        return false;

    }

    public boolean swivelAboutLeft(double speed, double targetHeading) {

        double error     = driveSubsystem.getHeadingError(targetHeading);
        double diffError = error - previousError;
        previousError = error;

        HeadingStates headingState;
        if (Math.abs(previousError) > DriveConstants.TURN_TO_HEADING_CLOSE) {
            headingState = HeadingStates.FAR;
        }
        else {
            headingState = HeadingStates.CLOSE;
        }

        switch (headingState) {

        case FAR:
        default:

            driveSubsystem.setMotorSpeeds(0, -speed * Math.signum(error));
            break;

        case CLOSE:

            pTerm = DriveConstants.TURN_TO_HEADING_PID_KP * error;
            iTerm += DriveConstants.TURN_TO_HEADING_PID_KI * error;
            dTerm = DriveConstants.TURN_TO_HEADING_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            Math.max(Math.min(errorSignal, 1), -1);

            driveSubsystem.setMotorSpeeds(0, -errorSignal * Math.signum(error));

            break;

        }

        if (error <= DriveConstants.TURN_TO_HEADING_TOLERANCE) {
            return true;
        }
        return false;

    }

    public boolean swivelAboutRight(double speed, double targetHeading) {

        double error     = driveSubsystem.getHeadingError(targetHeading);
        double diffError = error - previousError;
        previousError = error;

        HeadingStates headingState;
        if (Math.abs(previousError) > DriveConstants.TURN_TO_HEADING_CLOSE) {
            headingState = HeadingStates.FAR;
        }
        else {
            headingState = HeadingStates.CLOSE;
        }

        switch (headingState) {

        case FAR:
        default:

            driveSubsystem.setMotorSpeeds(speed * Math.signum(error), 0);
            break;

        case CLOSE:

            pTerm = DriveConstants.TURN_TO_HEADING_PID_KP * error;
            iTerm += DriveConstants.TURN_TO_HEADING_PID_KI * error;
            dTerm = DriveConstants.TURN_TO_HEADING_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            Math.max(Math.min(errorSignal, 1), -1);

            driveSubsystem.setMotorSpeeds(errorSignal * Math.signum(error), 0);

            break;

        }

        if (error <= DriveConstants.TURN_TO_HEADING_TOLERANCE) {
            return true;
        }
        return false;

    }



}
