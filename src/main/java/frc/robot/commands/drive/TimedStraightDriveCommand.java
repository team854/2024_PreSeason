package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedStraightDriveCommand extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         time, leftSpeed, rightSpeed;
    private boolean        brakeAtEnd;

    private long           initialTime, currentTime;
    private long           runTime     = 0;

    private double         K_p         = 0;
    private double         K_i         = 0;
    private double         K_d         = 0;

    private double         errorSignal = 0;
    private double         pTerm;
    private double         iTerm;
    private double         dTerm;



    public TimedStraightDriveCommand(double time, double leftSpeed, double rightSpeed, DriveSubsystem driveSubsystem) {
        this.time           = time;
        this.leftSpeed      = leftSpeed;
        this.rightSpeed     = rightSpeed;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        initialTime = System.currentTimeMillis();

        errorSignal = driveSubsystem.getYaw();

    }

    @Override
    public void execute() {
        // executes every 20ms
        double currentError = driveSubsystem.getYaw();
        double diffError    = currentError - errorSignal;

        pTerm        = K_p * errorSignal;
        iTerm       += K_i * errorSignal;
        dTerm       += K_d * diffError;

        errorSignal  = pTerm + iTerm + dTerm;

        leftSpeed   -= errorSignal;
        rightSpeed  += errorSignal;

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
