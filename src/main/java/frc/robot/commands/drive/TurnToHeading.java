package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeading extends LoggingCommand {

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
    }

    @Override
    public void execute() {

        // executes every 20ms
        currentHeading = driveSubsystem.getYaw();
        degreeDiff     = targetHeading - currentHeading;


        if ((degreeDiff > 0) && (degreeDiff < 180)) {
            // turn cw
        }
        else if ((degreeDiff > 0) && (degreeDiff >= 180)) {
            // turn ccw
        }
        // add the rest of them later

    }

    @Override
    public boolean isFinished() {

        // executes every 20ms
        degreeDiff = currentHeading - targetHeading;

        // FIXME add isFinished logic
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }
    }

}
