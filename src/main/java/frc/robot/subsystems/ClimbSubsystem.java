package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.operator.GameController;


public class ClimbSubsystem extends SubsystemBase {


    VictorSP                    leftClimber      = new VictorSP(ClimbConstants.LEFT_CLIMB_PORT);
    VictorSP                    rightClimber     = new VictorSP(ClimbConstants.RIGHT_CLIMB_PORT);

    double                      leftSpeed;
    double                      rightSpeed;

    int                         leftCount;
    int                         rightCount;


    public final GameController driverController = new GameController(
        OperatorConstants.DRIVER_CONTROLLER_PORT,
        OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);


    public ClimbSubsystem() {

        leftClimber.setInverted(ClimbConstants.LEFT_CLIMBER_REVERSED);
        rightClimber.setInverted(ClimbConstants.RIGHT_CLIMBER_REVERSED);

    }

    @Override
    public void periodic() {

        checkClimberSafety();
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);

        SmartDashboard.putNumber("Left Encoder Count", this.leftCount);
        SmartDashboard.putNumber("Right Encoder Count", this.rightCount);
        SmartDashboard.putNumber("Left Displacement (cm)", getDispLeftCM());
        SmartDashboard.putNumber("Right Displacement (cm)", getDispRightCM());
        SmartDashboard.putNumber("Left speed", this.leftSpeed);
        SmartDashboard.putNumber("Right speed", this.rightSpeed);


    }


    // encoder methods

    public double getLeftEncoderCount() {
        // find how to do encoders
        this.leftCount = leftClimber.getChannel();
        return leftCount;
    }

    public double getRightEncoderCount() {
        // find how to do encoders
        this.rightCount = rightClimber.getChannel();
        return rightClimber.get();
    }

    public double getDispLeftCM() {
        return getLeftEncoderCount() / ClimbConstants.ENCODER_COUNTS_PER_REVOLUTION * ClimbConstants.REVOLUTIONS_TO_CM;
    }

    public double getDispRightCM() {
        return getRightEncoderCount() / ClimbConstants.ENCODER_COUNTS_PER_REVOLUTION * ClimbConstants.REVOLUTIONS_TO_CM;
    }

    public double getDispErrorLeft(double targetDisp) {
        return targetDisp - getDispLeftCM();
    }

    public double getDispErrorRight(double targetDisp) {
        return targetDisp - getDispRightCM();
    }

    public void setLeftSpeed(double leftSpeed) {
        this.leftSpeed = leftSpeed;
        leftClimber.set(leftSpeed);
    }

    public void setRightSpeed(double rightSpeed) {
        this.rightSpeed = rightSpeed;
        rightClimber.set(rightSpeed);
    }

    public void checkClimberSafety() {
        if (getDispLeftCM() > ClimbConstants.MAXIMUM_DISPLACEMENT + 2) {
            this.leftSpeed = 0;
            setLeftSpeed(0);
        }
        if (getDispRightCM() > ClimbConstants.MAXIMUM_DISPLACEMENT + 2) {
            this.rightSpeed = 0;
            setRightSpeed(0);
        }
    }

}