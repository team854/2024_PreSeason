package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeading extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         targetHeading;
    private double         currentHeading;
    private double         degreeDiff;
    private double         speed;
    private boolean        brakeAtEnd;


    public TurnToHeading(double speed, double targetHeading, boolean brakeAtEnd, DriveSubsystem driveSubsystem) {
        this.speed          = speed;
        this.brakeAtEnd     = brakeAtEnd;
        this.targetHeading  = targetHeading;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", brake: " + brakeAtEnd + ", target heading: " + targetHeading;
        logCommandStart(commandParms);

        currentHeading = driveSubsystem.getYaw();

        double cwDist = targetHeading - currentHeading;
        if (cwDist < 0) {
            cwDist = cwDist + 360;
        }
        double ccwDist = currentHeading - targetHeading;
        if (ccwDist < 0) {
            ccwDist = ccwDist + 360;
        }

        degreeDiff = cwDist;
    }

    @Override
    public void execute() {

        // executes every 20ms

        currentHeading = driveSubsystem.getYaw();

        double cwDist = targetHeading - currentHeading;
        if (cwDist < 0) {
            cwDist = cwDist + 360;
        }
        double ccwDist = currentHeading - targetHeading;
        if (ccwDist < 0) {
            ccwDist = ccwDist + 360;
        }


        if (cwDist >= ccwDist) {
            degreeDiff = cwDist;
            driveSubsystem.setMotorSpeeds(0.5, -0.5);
        }
        else {
            degreeDiff = ccwDist;
            driveSubsystem.setMotorSpeeds(0.5, -0.5);
        }
        // add the rest of them later

    }

    @Override
    public boolean isFinished() {

        // executes every 20ms

        if (degreeDiff <= Constants.DriveConstants.HEADING_ERROR_BUFFER) {
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
