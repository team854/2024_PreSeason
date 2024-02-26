package frc.robot.commands.sysId;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DynamicForwardCommand extends LoggingCommand {

    DriveSubsystem                     m_driveSubsystem;

    Measure<Time>                      m_timeoutS;
    Timer                              m_timer;

    MutableMeasure<Distance>           m_distance;
    MutableMeasure<Voltage>            m_appliedVoltage;
    MutableMeasure<Velocity<Distance>> m_velocity;

    MutableMeasure<Voltage>            m_outputVolts = mutable(Volts.of(0));
    Consumer<State>                    m_recordState;

    Mechanism                          m_mechanism;
    Config                             m_config;

    String                             endMsg;

    public DynamicForwardCommand() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interupted) {

    }

}
