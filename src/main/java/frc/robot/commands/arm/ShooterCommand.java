package frc.robot.commands.arm;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ShooterCommand extends LoggingCommand {

    ArmSubsystem armSubsystem;

    double       speed;

    double       initTime;
    double       currTime;
    double       timeoutMS;

    String       reason;


    public ShooterCommand(double speed, double timeoutMS, ArmSubsystem armSubsystem) {

        this.speed        = speed;
        this.timeoutMS    = timeoutMS;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", timeout time (ms): " + timeoutMS;
        logCommandStart(commandParms);

        initTime = System.currentTimeMillis();



    }

    @Override
    public void execute() {

        armSubsystem.intakeSetSpeed(-speed);

    }

    @Override
    public boolean isFinished() {

        currTime = System.currentTimeMillis();

        if (currTime - initTime >= timeoutMS) {
            reason = "timeout of " + timeoutMS + " ms extends";
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
