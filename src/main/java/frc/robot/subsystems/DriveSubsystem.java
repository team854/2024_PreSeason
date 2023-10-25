package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final TalonSRX      leftPrimaryMotor         = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT);
    private final TalonSRX      leftFollowerMotor        = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT + 1);

    // The motors on the right side of the drive.
    private final TalonSRX      rightPrimaryMotor        = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT);
    private final TalonSRX      rightFollowerMotor       = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT + 1);

    private final DigitalInput  targetSensor             = new DigitalInput(0);

    // Conversion from volts to distance in cm
    // Volts distance
    // 0.12 30.5 cm
    // 2.245 609.6 cm
    private final AnalogInput   ultrasonicDistanceSensor = new AnalogInput(0);

    private static final double ULTRASONIC_M             = (609.6 - 30.5) / (2.245 - .12);
    private static final double ULTRASONIC_B             = 609.6 - ULTRASONIC_M * 2.245;


    private double              leftSpeed                = 0;
    private double              rightSpeed               = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        leftFollowerMotor.setNeutralMode(NeutralMode.Brake);

        leftFollowerMotor.follow(leftPrimaryMotor);


        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        rightFollowerMotor.setNeutralMode(NeutralMode.Brake);

        rightFollowerMotor.follow(rightPrimaryMotor);

    }

    public double getUltrasonicDistanceCm() {

        double ultrasonicVoltage = ultrasonicDistanceSensor.getVoltage();

        double distanceCm        = ULTRASONIC_M * ultrasonicVoltage + ULTRASONIC_B;

        return Math.round(distanceCm);
    }

    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        leftPrimaryMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightPrimaryMotor.set(ControlMode.PercentOutput, rightSpeed);

        // NOTE: The follower motors are set to follow the primary
        // motors
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    public boolean isTargetDetected() {
        return !targetSensor.get();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left  Motor", leftSpeed);

        SmartDashboard.putNumber("Ultrasonic Voltage", ultrasonicDistanceSensor.getVoltage());
        SmartDashboard.putNumber("Ultrasonic Distance (cm)", getUltrasonicDistanceCm());
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName())
            .append(" [").append(leftSpeed)
            .append(',').append(rightSpeed).append(']')
            .append(" ultrasonic dist ").append(getUltrasonicDistanceCm());

        return sb.toString();
    }

}
