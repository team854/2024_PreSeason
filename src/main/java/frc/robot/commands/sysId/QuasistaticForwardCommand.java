package frc.robot.commands.sysId;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class QuasistaticForwardCommand extends LoggingCommand {

    DriveSubsystem                     m_driveSubsystem;

    double                             m_timeoutS;
    double                             m_initTime;
    double                             m_currTime;

    MutableMeasure<Distance>           m_distance       = mutable(Meters.of(0));
    MutableMeasure<Voltage>            m_appliedVoltage = mutable(Volts.of(0));
    MutableMeasure<Velocity<Distance>> m_velocity       = mutable(MetersPerSecond.of(0));

    MutableMeasure<Voltage>            m_outputVolts    = mutable(Volts.of(0));
    Consumer<State>                    m_recordState;

    Mechanism                          m_mechanism;
    Config                             m_config;

    String                             endMsg;

    Consumer<SysIdRoutineLog>          m_logging        = log -> {
                                                            // Record a frame for the left motors.
                                                            // Since these share an encoder, we
                                                            // consider the entire group to be one
                                                            // motor.
                                                            log.motor("drive-left")
                                                                .voltage(m_appliedVoltage
                                                                    .mut_replace(m_driveSubsystem.m_leftPrimaryMotor.get()
                                                                        * RobotController.getBatteryVoltage(), Volts))
                                                                .linearPosition(m_distance
                                                                    .mut_replace(m_driveSubsystem.getLeftDistanceM(), Meters))
                                                                .linearVelocity(m_velocity.mut_replace(
                                                                    m_driveSubsystem.getLeftVelocityMPS(), MetersPerSecond));
                                                            // Record a frame for the right motors.
                                                            // Since these share an encoder, we
                                                            // consider the entire group to be one
                                                            // motor.
                                                            log.motor("drive-right")
                                                                .voltage(m_appliedVoltage
                                                                    .mut_replace(m_driveSubsystem.m_rightPrimaryMotor.get()
                                                                        * RobotController.getBatteryVoltage(), Volts))
                                                                .linearPosition(m_distance
                                                                    .mut_replace(m_driveSubsystem.getRightDistanceM(), Meters))
                                                                .linearVelocity(m_velocity.mut_replace(
                                                                    m_driveSubsystem.getRightVelocityMPS(), MetersPerSecond));
                                                        };

    public QuasistaticForwardCommand(double timeoutS, Mechanism mechanism, Config config, DriveSubsystem driveSubsystem) {

        this.m_timeoutS       = timeoutS;
        this.m_mechanism      = mechanism;
        this.m_config         = config;
        this.m_driveSubsystem = driveSubsystem;


        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        String commandParms = "timout (s): " + m_timeoutS;
        logCommandStart(commandParms);

        m_initTime = System.currentTimeMillis();

    }

    @Override
    public void execute() {

        m_currTime = System.currentTimeMillis();

        // m_mechanism.m_drive.accept(
        double leftSpeed  = (m_currTime - m_initTime) / 1000d * m_config.m_rampRate.in(Volts.per(Second))
            / RobotController.getBatteryVoltage();
        double rightSpeed = (m_currTime - m_initTime) / 1000d * m_config.m_rampRate.in(Volts.per(Second))
            / RobotController.getBatteryVoltage();

        m_driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

        // m_driveSubsystem.setMotorSpeeds(0.2, 0.2);



    }

    @Override
    public boolean isFinished() {

        m_currTime = System.currentTimeMillis();

        if (isTimeoutExceeded(m_timeoutS)) {
            endMsg = "Timeout of " + m_timeoutS + " seconds exceeded";
            return true;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        // m_config.m_recordState.accept(State.kNone);

        m_driveSubsystem.setMotorSpeeds(0, 0);
        logCommandEnd(interrupted, endMsg);

        m_config.m_recordState.accept(State.kQuasistaticForward);
        m_logging.accept(new SysIdRoutine(m_config, m_mechanism));

    }

}
