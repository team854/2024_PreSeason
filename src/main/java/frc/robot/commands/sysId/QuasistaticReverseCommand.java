package frc.robot.commands.sysId;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class QuasistaticReverseCommand extends LoggingCommand {

    DriveSubsystem                     m_driveSubsystem;

    double                             m_timeoutS;
    double                             m_initTime;
    double                             m_currTime;

    MutableMeasure<Distance>           m_distance;
    MutableMeasure<Voltage>            m_appliedVoltage;
    MutableMeasure<Velocity<Distance>> m_velocity;

    MutableMeasure<Voltage>            m_outputVolts = mutable(Volts.of(0));

    Mechanism                          m_mechanism;
    Config                             m_config;

    String                             endMsg;

    public QuasistaticReverseCommand(double timeoutS, Mechanism mechanism, Config config, DriveSubsystem driveSubsystem) {

        this.m_timeoutS       = timeoutS;
        this.m_mechanism      = mechanism;
        this.m_config         = config;
        this.m_driveSubsystem = driveSubsystem;


        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        String commandParms = "";
        logCommandStart(commandParms);

        m_initTime = System.currentTimeMillis();

    }

    @Override
    public void execute() {

        /*
         * m_mechanism.m_drive.accept(
         * m_outputVolts.mut_replace(-1 * m_timer.get() * m_config.m_rampRate.in(Volts.per(Second)),
         * Volts));
         * 
         * m_config.m_recordState.accept(State.kQuasistaticReverse);
         * m_mechanism.m_log.accept(new SysIdRoutine(m_config, m_mechanism));
         */

    }

    @Override
    public boolean isFinished() {

        if ((m_currTime - m_initTime) / 1000d > m_timeoutS) {
            endMsg = "Timeout of " + m_timeoutS + " seconds exceeded";
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        // m_mechanism.m_drive.accept(Volts.of(0));
        // m_config.m_recordState.accept(State.kNone);

        // m_mechanism.m_log.accept(new SysIdRoutine(m_config, m_mechanism));

        logCommandEnd(interrupted, endMsg);

    }

}
