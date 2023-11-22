package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.operator.OperatorInput;
import frc.robot.operator.OperatorInput.Stick;
import frc.robot.operator.OperatorInput.Axis;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand {

    private final DriveSubsystem driveSubsystem;
    private final OperatorInput  driverController;


    public DefaultDriveCommand(OperatorInput driverController, DriveSubsystem driveSubsystem) {

        this.driveSubsystem     = driveSubsystem;
        this.driverController   = driverController;

        //addRequirements(driveSubsystem);

    }

    public initialize() {

    }

    public void execute() {

        // forwards/backwards speed
        double       speed   = driverController.getDriverControllerAxis(Stick.LEFT, Axis.Y);
        // turn speed
        final double rawTurn = driverController.getDriverControllerAxis(Stick.RIGHT, Axis.X);

        final double leftSpeed;
        final double rightSpeed;

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Turn", rawTurn);

        //do math

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);


    }

    public boolean isFinished() {
        return false;
    }

}

