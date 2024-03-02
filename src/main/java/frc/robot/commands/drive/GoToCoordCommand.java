package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class GoToCoordCommand extends LoggingCommand {

    private final DriveSubsystem driveSubsystem;

    double                       x;
    double                       y;

    double                       veloX;
    double                       veloY;
    double                       angVelo;
    double                       leftVelo;
    double                       rightVelo;

    // Chassis movingFrameSpeeds;
    DifferentialDriveKinematics  wheelSpeeds;



    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public GoToCoordCommand(double x, double y, DriveSubsystem driveSubsystem) {

        this.x              = x;
        this.y              = y;
        this.driveSubsystem = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        String commandParms = "x = " + x + ", y = " + y;

        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double ang             = driveSubsystem.gyroSensorAhrs.getYaw();
        double leftDistMeters  = driveSubsystem.getLeftEncoder() / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION * Math.PI
            * DriveConstants.ROBOT_WHEEL_DIAMETER_M;
        double rightDistMeters = driveSubsystem.getRightEncoder() / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION * Math.PI
            * DriveConstants.ROBOT_WHEEL_DIAMETER_M;
        ;
        // DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(ang, leftDistMeters,
        // rightDistMeters);


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }


}