package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// This command doesn't perfectly account for encoders being overmeasured because of slight correction turns that the PID needs to make
public class MeasuredStraightDriveCommand extends BaseDriveCommand {

    private DriveSubsystem driveSubsystem;
    private double         dist;
    private double         speed;
    private boolean        brakeAtEnd;

    private double         initialLeftEncoder;
    private double         initialRightEncoder;
    private double         cmToEncoderCounts;

    private String         reason;



    public MeasuredStraightDriveCommand(double dist, double speed, boolean brakeAtEnd, DriveSubsystem driveSubsystem) {
        super(driveSubsystem);

        this.dist       = dist;
        this.speed      = speed;
        this.brakeAtEnd = brakeAtEnd;
    }

    @Override
    public void initialize() {

        String commandParms = "distance (cm): " + dist + ", speed: " + speed + ", brake: "
            + brakeAtEnd;

        logCommandStart(commandParms);

        initialLeftEncoder  = driveSubsystem.getLeftEncoder();
        initialRightEncoder = driveSubsystem.getRightEncoder();

        cmToEncoderCounts   = Math.abs(dist * DriveConstants.ENCODER_COUNTS_PER_CM);

    }

    @Override
    public void execute() {

        driveStraight(speed);

    }

    @Override
    public boolean isFinished() {

        double currentLeftEncoder   = driveSubsystem.getLeftEncoder();
        double currentRigthtEncoder = driveSubsystem.getRightEncoder();

        double countedLeft          = Math.abs(currentLeftEncoder - initialLeftEncoder);
        double countedRight         = Math.abs(currentRigthtEncoder - initialRightEncoder);

        if ((countedLeft + countedRight) / 2 >= cmToEncoderCounts) {
            reason = "passed a distance of " + dist + " cm";
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
