package frc.robot.commands.arm;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends LoggingCommand {

    ArmSubsystem armSubsystem;

    double       speed;

    double       initTime;
    double       currTime;
    double       timeout;

    String       reason;


    public IntakeCommand(double speed, double timeout, ArmSubsystem armSubsystem) {

        this.speed        = speed;
        this.timeout      = timeout;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", timeout time (ms): " + timeout;
        logCommandStart(commandParms);

        initTime = System.currentTimeMillis();



    }

    @Override
    public void execute() {

        armSubsystem.intakeSetSpeed(speed);

    }

    @Override
    public boolean isFinished() {

        currTime = System.currentTimeMillis();

        if (currTime - initTime >= timeout) {
            reason = "timeout of " + timeout + " ms extends";
            return true;
        }

        if (armSubsystem.isLoaded()) {
            reason = "filled intake";
            return true;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.intakeSetSpeed(0);


        setFinishReason(reason);
        logCommandEnd(interrupted);

    }



}
