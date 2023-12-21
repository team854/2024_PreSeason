package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCommand extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         time, leftSpeed, rightSpeed;
    private boolean        brakeAtEnd;



    public TimedDriveCommand(double time, double leftSpeed, double rightSpeed, boolean brakeAtEnd,
        DriveSubsystem driveSubsystem) {

        this.time           = time;
        this.leftSpeed      = leftSpeed;
        this.rightSpeed     = rightSpeed;
        this.driveSubsystem = driveSubsystem;
        this.brakeAtEnd     = brakeAtEnd;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        String commandParms = "time (ms): " + time + ", left speed: " + leftSpeed + ", right speed: " + rightSpeed + ", brake: "
            + brakeAtEnd;
        logCommandStart(commandParms);

    }

    @Override
    public void execute() {

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



        logCommandEnd(interrupted);
        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }


    }

}
