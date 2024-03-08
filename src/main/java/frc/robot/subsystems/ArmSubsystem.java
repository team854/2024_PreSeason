package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.operator.GameController;


public class ArmSubsystem extends SubsystemBase {


    CANSparkMax                 pivot            = new CANSparkMax(ArmConstants.PIVOT_PORT,
        CANSparkLowLevel.MotorType.kBrushless);
    WPI_TalonSRX                intakeLower      = new WPI_TalonSRX(ArmConstants.INTAKE_LOWER_PORT);
    WPI_TalonSRX                intakeHigher     = new WPI_TalonSRX(ArmConstants.INTAKE_HIGHER_PORT);
    DigitalInput                proximitySensor  = new DigitalInput(ArmConstants.PROXIMITY_PORT);

    boolean                     loaded;

    double                      currAnglePivot;
    double                      encoderCountPivot;

    double                      pivotRotSpeed;
    double                      intakeLowerSpeed;
    double                      intakeHigherSpeed;


    public final GameController driverController = new GameController(
        OperatorConstants.DRIVER_CONTROLLER_PORT,
        OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);


    public ArmSubsystem() {

        // The arm pivot motor needs to be inverted
        pivot.setInverted(true);


        pivot.getEncoder().setPosition(0); // below level

        this.currAnglePivot = getAnglePivot();

        pivot.setIdleMode(IdleMode.kBrake);

    }

    @Override
    public void periodic() {

        // Some simple safety code
        checkArmSafety();
        pivotRotSetSpeed(pivotRotSpeed);

        SmartDashboard.putNumber("Pivot Motor Speed", this.pivotRotSpeed);
        SmartDashboard.putNumber("Lower Intake Motor Speed", this.intakeLowerSpeed);
        SmartDashboard.putNumber("Higher Intake Motor Speed", this.intakeHigherSpeed);

        SmartDashboard.putNumber("Pivot encoder count", this.encoderCountPivot);
        SmartDashboard.putNumber("Arm Angle", getAnglePivot());

        SmartDashboard.putBoolean("Loaded", this.loaded);



    }

    private void checkArmSafety() {
        if (this.pivotRotSpeed > 0 && getAnglePivot() > ArmConstants.MAX_ARM_PIVOT_ANGLE) {
            this.pivotRotSpeed = 0;
        }
        if (this.pivotRotSpeed < 0 && getAnglePivot() < ArmConstants.MIN_ARM_PIVOT_ANGLE) {
            this.pivotRotSpeed = 0;
        }

    }

    // pivot methods

    public double getEncoderCountPivot() {

        // floor = 0;
        // 0 deg = -2.4
        // 90 deg = -11.71
        // max = 17

        // Starting position is zero encoder counts which is acutally
        // a -2.4 count from offset
        this.encoderCountPivot = pivot.getEncoder().getPosition() - 2.4;
        return this.encoderCountPivot;
    }

    public double getAnglePivot() {

        this.currAnglePivot = getEncoderCountPivot() / ArmConstants.PIVOT_ARM_ENCODER_COUNT_PER_ROTATION * 360;
        return this.currAnglePivot;
    }

    public double getAngleErrorPivot(double targetAngle) {
        return targetAngle - getAnglePivot();
    }

    public void pivotRotSetSpeed(double speed) {
        this.pivotRotSpeed = speed;

        checkArmSafety();

        pivot.set(pivotRotSpeed);
    }

    // intake methods

    public void intakeSetSpeed(double speed) {
        this.intakeLowerSpeed  = speed;
        this.intakeHigherSpeed = speed;
        intakeLower.set(speed);
        intakeHigher.set(speed);
    }

    // proximity sensor methods

    public boolean isLoaded() {
        this.loaded = proximitySensor.get();
        return loaded;
    }


}