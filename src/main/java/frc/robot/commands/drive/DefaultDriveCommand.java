package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.LoggingCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class DefaultDriveCommand extends LoggingCommand {

    private final DriveSubsystem             driveSubsystem;
    private final OperatorInput              operatorInput;
    private final LightsSubsystem            lightsSubsystem;

    private final SendableChooser<DriveMode> driveModeChooser;

    double                                   veloX;
    double                                   veloY;
    double                                   angVelo;
    double                                   leftSpeed;
    double                                   rightSpeed;
    ChassicSpeeds                            movingFrameSpeeds;
    DifferentialDriveKinematics              wheelSpeeds;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(OperatorInput operatorInput, SendableChooser<DriveMode> driveModeChooser,
        DriveSubsystem driveSubsystem, LightsSubsystem lightsSubsystem) {

        this.operatorInput    = operatorInput;
        this.driveModeChooser = driveModeChooser;
        this.driveSubsystem   = driveSubsystem;
        this.lightsSubsystem  = lightsSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart("default drive command");
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
            angVelo = Math.asin(operatorInput.getLeftX());

            movingFrameSpeeds = new ChassisSpeeds(veloX, veloY, angVelo);
            wheelSpeeds = new DifferentialDriveKinematics(DriveConstants.WIDTH_WHEEL_TO_WHEEL);

            leftSpeed = wheelSpeeds.leftMetersPerSecond();
            rightSpeed = wheelSpeeds.rightMetersPerSecond();

            if (boost) {
                driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
            }
            else {
                driveSubsystem.setMotorSpeeds(0.5 * leftSpeed, 0.5 * rightSpeed);
            }

            break;

        case DUAL_STICK_ARCADE:

            veloX = DriveConstants.MAX_WHEEL_SPEED_MPS * operatorInput.getLeftY();
            veloY = 0;
            angVelo = Math.asin(operatorInput.getRightX());

            movingFrameSpeeds = new ChassisSpeeds(veloX, veloY, angVelo);
            wheelSpeeds = new DifferentialDriveKinematics(DriveConstants.WIDTH_WHEEL_TO_WHEEL);

            leftSpeed = wheelSpeeds.leftMetersPerSecond();
            rightSpeed = wheelSpeeds.rightMetersPerSecond();

            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);


            break;

        case TANK:
        default:

            leftSpeed = operatorInput.getLeftSpeed();
            rightSpeed = operatorInput.getRightSpeed();
            if (boost) {
                lightsSubsystem.ledStick(boost, driveMode);
                driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
            }
            else {
                // If not in boost mode, then divide the motors speeds in half
                lightsSubsystem.ledStick(boost, driveMode);
                driveSubsystem.setMotorSpeeds(leftSpeed / 2.0, rightSpeed / 2.0);
            }

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