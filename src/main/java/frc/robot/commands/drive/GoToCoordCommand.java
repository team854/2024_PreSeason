package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommandBase;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.DriveSubsystem;

public class GoToCoordCommand extends LoggingCommandBase {

    private final DriveSubsystem             driveSubsystem;
    private final OperatorInput              operatorInput;
    private final SendableChooser<DriveMode> driveModeChooser;

    double                                   x;
    dobule                                   y;

    double                                   veloX;
    double                                   veloY;
    double                                   angVelo;
    double                                   leftVelo;
    double                                   rightVelo;

    Chassis                                  movingFrameSpeeds;
    DifferentialDriveKinematics              wheelSpeeds;



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

        double ang             = driveSubsystem.gyroSensorAhrs();
        double leftDistMeters  = driveSubsystem.getLeftEncoder() / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION * Math.PI
            * DriveConstants.ROBOT_WHEEL_DIAMETER_M;
        double rightDistMeters = driveSubsystem.getRightEncoder() / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION * Math.PI
            * DriveConstants.ROBOT_WHEEL_DIAMETER_M;
        ;
        DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(ang, leftDistMeters, rightDistMeters);


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }


}