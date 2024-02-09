package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

// This command doesn't perfectly account for encoders being overmeasured because of slight correction turns that the PID needs to make
public class MeasuredStraightDriveCommand extends LoggingCommand {

    private DriveSubsystem driveSubsystem;
    private double         dist;
    private double         speed;
    private boolean        brakeAtEnd;



    private double         errorSignal;
    private double         previousError;
    private double         targetHeading;
    private double         pTerm;
    private double         iTerm = 0;
    private double         dTerm;
    private double         initialLeftEncoder;
    private double         initialRightEncoder;
    private double         cmToEncoderUnits;



    public MeasuredStraightDriveCommand(double dist, double speed, boolean brakeAtEnd, double targetHeading,
        DriveSubsystem driveSubsystem) {
        this.dist           = dist;
        this.speed          = speed;
        this.targetHeading  = targetHeading;
        this.driveSubsystem = driveSubsystem;
        this.brakeAtEnd     = brakeAtEnd;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        String commandParms = "distance (cm): " + dist + ", speed: " + speed + ", target heading: " + targetHeading + ", brake: "
            + brakeAtEnd;

        logCommandStart(commandParms);

        initialLeftEncoder  = driveSubsystem.getLeftEncoder();
        initialRightEncoder = driveSubsystem.getRightEncoder();

        cmToEncoderUnits    = dist * frc.robot.Constants.DriveConstants.ENCODER_COUNTS_PER_REVOLUTION;

    }

    @Override
    public void execute() {

        // executes every 20ms
        double currentError = driveSubsystem.getHeadingError(targetHeading);
        double diffError    = currentError - previousError;
        previousError  = currentError;

        pTerm          = DriveConstants.HEADING_PID_KP * currentError;
        iTerm         += DriveConstants.HEADING_PID_KI * diffError;
        dTerm          = DriveConstants.HEADING_PID_KD * diffError;


        errorSignal    = pTerm + iTerm + dTerm;

        double leftSpeed  = Math.min(Math.max(speed - errorSignal, -1.0), 1.0);
        double rightSpeed = Math.min(Math.max(speed + errorSignal, -1.0), 1.0);

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

    }

    @Override
    public boolean isFinished() {

        double currentLeftEncoder   = driveSubsystem.getLeftEncoder();
        double currentRigthtEncoder = driveSubsystem.getRightEncoder();

        double countedLeft          = currentLeftEncoder - initialLeftEncoder;
        double countedRight         = currentRigthtEncoder - initialRightEncoder;

        if ((countedLeft + countedRight) / 2 >= cmToEncoderUnits) {
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
