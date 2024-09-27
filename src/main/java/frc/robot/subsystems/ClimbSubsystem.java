package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.operator.GameController;
import frc.robot.operator.OperatorInput;


public class ClimbSubsystem extends SubsystemBase {

    final OperatorInput         operatorInput;

    VictorSP                    leftClimber      = new VictorSP(ClimbConstants.LEFT_CLIMB_PORT);
    VictorSP                    rightClimber     = new VictorSP(ClimbConstants.RIGHT_CLIMB_PORT);

    double                      leftSpeed;
    double                      rightSpeed;

    int                         leftEncoder      = 0;
    int                         rightEncoder     = 0;

    DigitalInput                safetyLeft       = new DigitalInput(ClimbConstants.SAFETY_LEFT_PORT_DIO);
    DigitalInput                safetyRight      = new DigitalInput(ClimbConstants.SAFETY_RIGHT_PORT_DIO);


    public final GameController driverController = new GameController(
        OperatorConstants.DRIVER_CONTROLLER_PORT,
        OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);


    public ClimbSubsystem(OperatorInput operatorInput) {
        this.operatorInput = operatorInput;
        leftClimber.setInverted(ClimbConstants.LEFT_CLIMBER_REVERSED);
        rightClimber.setInverted(ClimbConstants.RIGHT_CLIMBER_REVERSED);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Left speed", this.leftSpeed);
        SmartDashboard.putNumber("Right speed", this.rightSpeed);

        SmartDashboard.putBoolean("Left is at limit", isLeftAtLimit());
        SmartDashboard.putBoolean("Right is at limit", isRightAtLimit());

    }

    public void setLeftSpeed(double leftSpeed) {
        this.leftSpeed = leftSpeed;
        if (isLeftAtLimit()) {
            this.leftSpeed = 0;
        }
        leftClimber.set(this.leftSpeed);
    }

    public void setRightSpeed(double rightSpeed) {
        this.rightSpeed = rightSpeed;
        if (isRightAtLimit()) {
            this.rightSpeed = 0;
            operatorInput.driverController.pulseRumble(1.0, 0.2);
        }
        rightClimber.set(this.rightSpeed);
    }


    public boolean isLeftAtLimit() {
        return !safetyLeft.get();
    }

    public boolean isRightAtLimit() {
        return !safetyRight.get();
    }

}