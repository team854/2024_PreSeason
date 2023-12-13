package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCommand extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         time, leftSpeed, rightSpeed;
    private boolean        brakeAtEnd;

    private long           initialTime, currentTime;
    private long           runTime = 0;


    public TimedDriveCommand(double time, double leftSpeed, double rightSpeed, DriveSubsystem driveSubsystem) {
        this.time           = time;
        this.leftSpeed      = leftSpeed;
        this.rightSpeed     = rightSpeed;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        initialTime = System.currentTimeMillis();

    }

    @Override
    public void execute() {
        // executes every 20ms
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

    }

    @Override
    public boolean isFinished() {
        currentTime = System.currentTimeMillis();
        runTime     = currentTime - initialTime;
        brakeAtEnd  = false;

        if (runTime <= time) {
            brakeAtEnd = false;
        }
        else {
            brakeAtEnd = true;
        }

        return brakeAtEnd;

    }

    @Override
    public void end(boolean brakeAtEnd) {
        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }
    }

}
