package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        DriveMode driveMode = driveModeChooser.getSelected();

        boolean   boost     = operatorInput.getBoost();

        switch (driveMode) {

        case SINGLE_STICK_ARCADE:
            // DNE
        case DUAL_STICK_ARCADE:

            double speed = operatorInput.getSpeed(driveMode);
            double turn = operatorInput.getTurn(driveMode) / 2;

            setMotorSpeedsArcade(speed, turn, boost);
            lightsSubsystem.ledStick(boost, driveMode);
            break;

        case TANK:
        default:

            double leftSpeed = operatorInput.getLeftSpeed();
            double rightSpeed = operatorInput.getRightSpeed();
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

    private void setMotorSpeedsArcade(double speed, double turn, boolean boost) {

        double maxSpeed   = 1.0;

        // The basic algorithm for arcade is to add the turn and the speed

        double leftSpeed  = speed + turn;
        double rightSpeed = speed - turn;

        // If the speed + turn exceeds the max speed, then keep the differential
        // and reduce the speed of the other motor appropriately

        if (Math.abs(leftSpeed) > maxSpeed || Math.abs(rightSpeed) > maxSpeed) {

            if (Math.abs(leftSpeed) > maxSpeed) {

                if (leftSpeed > 0) {
                    leftSpeed = maxSpeed;
                }
                else {
                    leftSpeed = -maxSpeed;
                }
                rightSpeed = leftSpeed - turn;

            }
            else {

                if (rightSpeed > 0) {
                    rightSpeed = maxSpeed;
                }
                else {
                    rightSpeed = -maxSpeed;
                }

                leftSpeed = rightSpeed + turn;
            }
        }

        if (!boost) {
            leftSpeed  /= 2.0;
            rightSpeed /= 2.0;
            turn       /= 2.0;
        }

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }


}