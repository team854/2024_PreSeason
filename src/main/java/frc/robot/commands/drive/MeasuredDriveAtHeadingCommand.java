package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// This command doesn't perfectly account for encoders being overmeasured because of slight correction turns that the PID needs to make
public class MeasuredDriveAtHeadingCommand extends BaseDriveCommand {

    private double  dist;
    private double  speed;
    private boolean brakeAtEnd;
    private double  targetHeading;

    private double  initialLeftEncoder;
    private double  initialRightEncoder;
    private double  cmToEncoderCounts;



    public MeasuredDriveAtHeadingCommand(double dist, double speed, boolean brakeAtEnd, double targetHeading,
        DriveSubsystem driveSubsystem) {

        super(driveSubsystem);

        this.dist          = dist;
        this.speed         = speed;
        this.targetHeading = targetHeading;
        this.brakeAtEnd    = brakeAtEnd;
    }

    @Override
    public void initialize() {

        String commandParms = "distance (cm): " + dist + ", speed: " + speed + ", target heading: " + targetHeading + ", brake: "
            + brakeAtEnd;

        logCommandStart(commandParms);

        cmToEncoderCounts   = Math.abs(dist * DriveConstants.ENCODER_COUNTS_PER_CM);

        initialLeftEncoder  = driveSubsystem.getLeftEncoder();
        initialRightEncoder = driveSubsystem.getRightEncoder();

    }

    @Override
    public void execute() {

        driveOnHeading(speed, targetHeading);

    }

    @Override
    public boolean isFinished() {

        double currentLeftEncoder   = driveSubsystem.getLeftEncoder();
        double currentRigthtEncoder = driveSubsystem.getRightEncoder();

        double countedLeft          = Math.abs(currentLeftEncoder - initialLeftEncoder);
        double countedRight         = Math.abs(currentRigthtEncoder - initialRightEncoder);

        if ((countedLeft + countedRight) / 2 >= cmToEncoderCounts) {
            setFinishReason("passed a distance of " + dist + " cm");
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
