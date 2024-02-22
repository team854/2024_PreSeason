package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {


    // The motors on the left side of the drive.
    private final WPI_VictorSPX m_leftPrimaryMotor   = new WPI_VictorSPX(DriveConstants.LEFT_MOTOR_PORT);
    private final WPI_TalonSRX  m_leftFollowerMotor  = new WPI_TalonSRX(DriveConstants.LEFT_MOTOR_PORT + 1);

    // The motors on the right side of the drive.
    private final WPI_VictorSPX m_rightPrimaryMotor  = new WPI_VictorSPX(DriveConstants.RIGHT_MOTOR_PORT);
    private final WPI_TalonSRX  m_rightFollowerMotor = new WPI_TalonSRX(DriveConstants.RIGHT_MOTOR_PORT + 1);



    // The gyro sensor
    private final AHRS                               m_gyroSensorAhrs           = new AHRS();

    // Ultrasonic sensor
    // Conversion from volts to distance in cm
    // Volts distance
    // 0.12 30.5 cm
    // 2.245 609.6 cm
    private final AnalogInput                        m_ultrasonicDistanceSensor = new AnalogInput(0);

    private static final double                      k_ultrasonicM              = (609.6 - 30.5) / (2.245 - .12);
    private static final double                      k_ultrasonicB              = 609.6 - k_ultrasonicM * 2.245;

    // Motor speeds
    private double                                   m_leftSpeed                = 0;
    private double                                   m_rightSpeed               = 0;

    // Drive
    private final DifferentialDrive                  m_drive                    = new DifferentialDrive(m_leftPrimaryMotor::set,
        m_rightPrimaryMotor::set);


    // SysId Routine
    private final MutableMeasure<Distance>           m_distance                 = mutable(Meters.of(0));
    private final MutableMeasure<Voltage>            m_appliedVoltage           = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity                 = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine                       m_sysIdRoutine             = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                                                                                            m_leftPrimaryMotor
                                                                                                .setVoltage(volts.in(Volts));
                                                                                            m_rightPrimaryMotor
                                                                                                .setVoltage(volts.in(Volts));
                                                                                        },
            log -> {
                // Record a frame for the left motors. Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftPrimaryMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getLeftDistanceM(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getLeftVelocityMPS(), MetersPerSecond));
                // Record a frame for the right motors. Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightPrimaryMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getRightDistanceM(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getRightVelocityMPS(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive")
            this));


    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {


        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        m_leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        m_leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        m_leftFollowerMotor.setNeutralMode(NeutralMode.Brake);

        m_leftFollowerMotor.follow(m_leftPrimaryMotor);


        m_rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        m_rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        m_rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        m_rightFollowerMotor.setNeutralMode(NeutralMode.Brake);

        m_rightFollowerMotor.follow(m_rightPrimaryMotor);

    }

    // Encoders
    public double getAverageEncoderCounts() {
        return (m_leftFollowerMotor.getSelectedSensorPosition(0) + m_rightFollowerMotor.getSelectedSensorPosition(0)) / 2;
    }

    public double getDistanceCm() {
        return getAverageEncoderCounts() * DriveConstants.CMS_PER_ENCODER_COUNT;
    }

    public double getLeftEncoder() {
        return m_leftFollowerMotor.getSelectedSensorPosition(0);
    }

    public double getRightEncoder() {
        return m_rightFollowerMotor.getSelectedSensorPosition(0);
    }

    public double getLeftDistanceM() {
        return getLeftEncoder() / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION * DriveConstants.ROBOT_WHEEL_CIRCUMFERENCE_CMS
            / 100d;
    }

    public double getRightDistanceM() {
        return getRightEncoder() / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION * DriveConstants.ROBOT_WHEEL_CIRCUMFERENCE_CMS
            / 100d;
    }

    public double getLeftVelocityMPS() {
        double initTime = System.currentTimeMillis();
        double currTime = System.currentTimeMillis();
        double diffTime = currTime - initTime;

        double initDist = getLeftDistanceM();
        double currDist = getLeftDistanceM();
        double diffDist = currDist - initDist;

        while (diffTime <= currTime) {
            currDist = getLeftDistanceM();
            currTime = System.currentTimeMillis();
            diffDist = currDist - initDist;
        }

        return diffDist / diffTime * 1000d;
    }

    public double getRightVelocityMPS() {
        double initTime = System.currentTimeMillis();
        double currTime = System.currentTimeMillis();
        double diffTime = currTime - initTime;

        double initDist = getRightDistanceM();
        double currDist = getRightDistanceM();
        double diffDist = currDist - initDist;

        while (diffTime <= currTime) {
            currDist = getRightDistanceM();
            currTime = System.currentTimeMillis();
            diffDist = currDist - initDist;
        }

        return diffDist / diffTime * 1000d;
    }


    // ultrasonic

    public double getUltrasonicDistanceCm() {

        double ultrasonicVoltage = m_ultrasonicDistanceSensor.getVoltage();

        // Use a straight line y = mx + b equation to convert voltage into cm.
        double distanceCm        = k_ultrasonicM * ultrasonicVoltage + k_ultrasonicB;

        return Math.round(distanceCm);
    }


    // motors

    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        this.m_leftSpeed  = leftSpeed;
        this.m_rightSpeed = rightSpeed;

        m_leftPrimaryMotor.set(ControlMode.PercentOutput, m_leftSpeed);
        m_rightPrimaryMotor.set(ControlMode.PercentOutput, m_rightSpeed);

        // NOTE: The follower motors are set to follow the primary motors
    }

    public void arcadeDriveCommand(double xSpeed, double zRot) {
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        m_drive.arcadeDrive(xSpeed, zRot);
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {

        /*
         * Update all dashboard values in the periodic routine
         */
        SmartDashboard.putNumber("Right Motor", m_rightSpeed);
        SmartDashboard.putNumber("Left Motor", m_leftSpeed);

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
        SmartDashboard.putNumber("Gyro Yaw", m_gyroSensorAhrs.getYaw());

    }

    @Override
    public String toString() {

        // Create an appropriate text readable string describing the state of the subsystem
        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName())
            .append(" [").append(Math.round(m_leftSpeed * 100.0d) / 100.0d)
            .append(',').append(Math.round(m_rightSpeed * 100.0d) / 100.0d).append(']')
            .append(" ultrasonic dist ").append(getUltrasonicDistanceCm());

        return sb.toString();

    }

    // returns the yaw, rounded to 1 decimal place
    // retuens in degrees from 0-360
    public double getHeading() {
        double yawAngle = (Math.round(m_gyroSensorAhrs.getYaw() * 10) / 10.0d) % 360;
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