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

    int                         leftEncoder      = 0;
    int                         rightEncoder     = 0;


    public final GameController driverController = new GameController(
        OperatorConstants.DRIVER_CONTROLLER_PORT,
        OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);


    public ClimbSubsystem() {

        leftClimber.setInverted(ClimbConstants.LEFT_CLIMBER_REVERSED);
        rightClimber.setInverted(ClimbConstants.RIGHT_CLIMBER_REVERSED);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Left speed", this.leftSpeed);
        SmartDashboard.putNumber("Right speed", this.rightSpeed);

    }

    public void setLeftSpeed(double leftSpeed) {
        this.leftSpeed = leftSpeed;
        leftClimber.set(leftSpeed);
    }

    public void setRightSpeed(double rightSpeed) {
        this.rightSpeed = rightSpeed;
        rightClimber.set(rightSpeed);
    }

    /*
     * public int getLeftEncoder() {
     * this.leftEncoder = leftClimber.getSelectedSensorPosition();
     * }
     * 
     * public int getRightEncoder() {
     * 
     * }
     * 
     * public double getRightDistance() {
     * 
     * }
     * 
     * public double getLeftDistance() {
     * 
     * }
     */

}