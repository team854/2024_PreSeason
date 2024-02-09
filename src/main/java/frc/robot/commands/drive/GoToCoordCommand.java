package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

        DriveMode driveMode = driveModeChooser.getSelected();

        boolean   boost     = operatorInput.getBoost();

        switch (driveMode) {

        case SINGLE_STICK_ARCADE:
            veloX = DriveConstants.MAX_WHEEL_SPEED_MPS * operatorInput.getLeftY();
            veloY = 0;
            angVelo = Math.PI / 2 * operatorInput.getLeftX();

            movingFrameSpeeds = new ChassisSpeeds(veloX, veloY, angVelo);
            wheelSpeeds = new DifferentialDriveKinematics(DriveConstants.WIDTH_WHEEL_TO_WHEEL);

            leftVelo = wheelSpeeds.leftMetersPerSecond();
            rightVelo = wheelSpeeds.rightMetersPerSecond();
            setMotorSpeedsArcade(leftVelo, rightVelo, boost);

            break;

        case DUAL_STICK_ARCADE:
        default:

            veloX = DriveConstants.MAX_WHEEL_SPEED_MPS * operatorInput.getLeftY();
            veloY = 0;
            angVelo = Math.asin(operatorInput.getRightX());

            movingFrameSpeeds = new ChassisSpeeds(veloX, veloY, angVelo);
            wheelSpeeds = new DifferentialDriveKinematics(DriveConstants.WIDTH_WHEEL_TO_WHEEL);

            leftVelo = wheelSpeeds.leftMetersPerSecond();
            rightVelo = wheelSpeeds.rightMetersPerSecond();
            if (boost) {
                driveSubsystem.setMotorSpeeds(leftVelo / DriveConstants.MAX_WHEEL_SPEED_MPS,
                    rightVelo / DriveConstants.MAX_WHEEL_SPEED_MPS);
            }
            else {
                driveSubsystem.setMotorSpeeds(0.5 * leftVelo / DriveConstants.MAX_WHEEL_SPEED_MPS, 0.5 *
                    rightVelo / DriveConstants.MAX_WHEEL_SPEED_MPS);
            }


            break;

        case TANK:

            leftVelo = operatorInput.getLeftSpeed();
            rightVelo = operatorInput.getRightSpeed();
            setMotorSpeedsArcade(leftVelo / DriveConstants.MAX_WHEEL_SPEED_MPS, rightVelo / DriveConstants.MAX_WHEEL_SPEED_MPS,
                boost);

            break;

        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default drive command never ends, but can be interrupted by other commands.
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }


}