package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {


    // The motors on the left side of the drive.
    private final CANSparkMax   leftPrimaryMotor         = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax   leftFollowerMotor        = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT + 1,
        CANSparkLowLevel.MotorType.kBrushless);

    // The motors on the right side of the drive.
    private final CANSparkMax   rightPrimaryMotor        = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax   rightFollowerMotor       = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT + 1,
        CANSparkLowLevel.MotorType.kBrushless);

    // The gyro sensor
    private final AHRS          gyroSensorAhrs           = new AHRS();

    // Ultrasonic sensor
    // Conversion from volts to distance in cm
    // Volts distance
    // 0.12 30.5 cm
    // 2.245 609.6 cm

    private final AnalogInput   ultrasonicDistanceSensor = new AnalogInput(0);

    private static final double ULTRASONIC_M             = (609.6 - 30.5) / (2.245 - .12);
    private static final double ULTRASONIC_B             = 609.6 - ULTRASONIC_M * 2.245;


    // Motor speeds
    private double              leftSpeed                = 0;
    private double              rightSpeed               = 0;


    // Odometry objects

    double        radPerSec     = 0;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(leftSpeed, rightSpeed, radPerSec);


    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {


        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        /*
         * leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
         * leftFollowerMotor.setNeutralMode(NeutralMode.Brake);
         */

        leftFollowerMotor.follow(leftPrimaryMotor);


        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        /*
         * rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
         * rightFollowerMotor.setNeutralMode(NeutralMode.Brake);
         */

        rightFollowerMotor.follow(rightPrimaryMotor);

    }

    // Encoders
    public double getAverageEncoderCounts() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getDistanceCm() {
        return getAverageEncoderCounts() * DriveConstants.CMS_PER_ENCODER_COUNT;
    }

    public double getLeftEncoder() {
        return leftFollowerMotor.getAlternateEncoder(DriveConstants.ENCODER_COUNTS_PER_REVOLUTION).getPosition();
    }

    public double getRightEncoder() {
        return rightFollowerMotor.getAlternateEncoder(DriveConstants.ENCODER_COUNTS_PER_REVOLUTION).getPosition();
    }



    public double getUltrasonicDistanceCm() {

        double ultrasonicVoltage = ultrasonicDistanceSensor.getVoltage();

        // Use a straight line y = mx + b equation to convert voltage into cm.
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

        leftPrimaryMotor.set(leftSpeed);
        rightPrimaryMotor.set(rightSpeed);

        // NOTE: The follower motors are set to follow the primary motors
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    @Override
    public void periodic() {

        /*
         * Update all dashboard values in the periodic routine
         */
        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left Motor", leftSpeed);

        // Update Encoder
        SmartDashboard.putNumber("Left Encoder", Math.round(getLeftEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Right Encoder", Math.round(getRightEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Average Encoder", Math.round(getAverageEncoderCounts() * 100) / 100d);
        SmartDashboard.putNumber("Distance (m)", Math.round(getDistanceCm() * 10) / 1000d);

        // Round the ultrasonic voltage to 2 decimals
        // SmartDashboard.putNumber("Ultrasonic Voltage",
        // Math.round(ultrasonicDistanceSensor.getVoltage() * 100.0d) / 100.0d);
        SmartDashboard.putNumber("Ultrasonic Distance (m)", getUltrasonicDistanceCm() / 100d);

        // Gets the yaw from the gyro sensor
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("Gyro Yaw", gyroSensorAhrs.getYaw());

    }

    @Override
    public String toString() {

        // Create an appropriate text readable string describing the state of the subsystem
        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName())
            .append(" [").append(Math.round(leftSpeed * 100.0d) / 100.0d)
            .append(',').append(Math.round(rightSpeed * 100.0d) / 100.0d).append(']')
            .append(" ultrasonic dist ").append(getUltrasonicDistanceCm());

        return sb.toString();

    }

    // returns the yaw, rounded to 1 decimal place
    // retuens in degrees from 0-360
    public double getHeading() {
        double yawAngle = (Math.round(gyroSensorAhrs.getYaw() * 10) / 10.0d) % 360;
        if (yawAngle < 0) {
            yawAngle += 360;
        }
        return yawAngle;
    }

    public double getHeadingError(double targetHeading) {
        double currentHeading = getHeading();
        double error          = targetHeading - currentHeading;
        if (error < 0) {
            error += 360;
        }
        if (error > 180) {
            error = error - 360;
        }

        return error;
    }

}