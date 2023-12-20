package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedStraightDriveCommand extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         time;
    private double         speed;
    private boolean        brakeAtEnd;

    // private long initialTime, currentTime;
    // private long runTime = 0;

    /*
     * FIXME: make a constant HEADING_PID_KP in DriveConstants
     * 
     * The proportional gain value translates an error in degrees (-180 to +180)
     * to a motor speed adjustment value (-1.0 to +1.0)
     * The units will be %motorOuput/degree. Something in the range of .01 to .05 might be more
     * appropriate.
     */



    private double errorSignal;
    private double previousError;
    private double currentHeading;
    private double targetHeading;
    private double pTerm;
    private double iTerm;
    private double dTerm;


    /**
     * FIXME are these the correct parameters?
     * For Drive straight, I would expect to pass in heading and speed (and time)
     * The robot would then track the passed in heading.
     * Alternately, you could start driving straight from the direction you are pointing, but that
     * is not really error tolerant. Plotting out the compass headings is more error tolerant.
     *
     * @param time
     * @param leftSpeed
     * @param rightSpeed
     * @param driveSubsystem
     */
    public TimedStraightDriveCommand(double time, double speed, boolean brakeAtEnd, double targetHeading,
        DriveSubsystem driveSubsystem) {
        this.time           = time;
        this.speed          = speed;
        this.driveSubsystem = driveSubsystem;
        this.brakeAtEnd     = brakeAtEnd;
        this.targetHeading  = targetHeading;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        /*
         * RM:
         * FIXME Always log the command start
         * logCommandStart(parms)
         */
        String commandParms = "time (ms): " + time + ", speed: " + speed + ", brake: "
            + brakeAtEnd + ", target heading (deg): " + targetHeading;

        logCommandStart(commandParms);

        // FIXME should the variable errorSignal be called initialHeading?
        currentHeading = driveSubsystem.getYaw();

    }

    @Override
    public void execute() {

        /*
         * FIXME: calculate heading error
         * Calculating the heading error is much more complex than the formula above.
         * The heading is in the range 0-360 (for both the target heading and the current heading)
         * The error should be in the range -180 to +180
         * 
         * Recommendation: make a getHeadingError(targetHeading) routine in the drive subsystem
         * that can pass back a heading error to any command that may need that information.
         */
        // executes every 20ms
        double currentError = driveSubsystem.getHeadingError(targetHeading);
        double diffError    = currentError - previousError;
        previousError  = currentError;

        /*
         * FIXME start with a simple proportional gain
         * 
         * HINT: Start with just a single, simple proportional gain - this should be sufficient for
         * tracking a compass heading
         */
        pTerm          = DriveConstants.HEADING_PID_KP * errorSignal;
        iTerm         += DriveConstants.HEADING_PID_KI * errorSignal;
        dTerm         += DriveConstants.HEADING_PID_KD * diffError;

        // FIXME error should be a local variable
        // this code below modifies the initial heading
        errorSignal    = pTerm + iTerm + dTerm;

        /*
         * FIXME calculate motor speeds
         * 
         * This code as written adjusts the passed in left and right speed on every loop?
         * HINT: use local variables, not the passed in values
         * 
         * HINT: the min/max for the motor speeds are -1.0 to +1.0. Do we want to
         * rotate at max speed, or is there some maximum rotational speed (max difference between
         * right and left motor speeds)?
         * Where should the max rotation speed be governed (here, or in DriveSubsystem)?
         */
        double leftSpeed  = speed - errorSignal;
        double rightSpeed = speed + errorSignal;

        driveSubsystem.setMotorSpeeds(Math.min(leftSpeed, 1.0), Math.min(rightSpeed, 1.0));

    }

    @Override
    public boolean isFinished() {

        // FIXME isFinished()
        // should return true or false - the brakeAtEnd should be passed into the command, see
        // TimedDrive command

        if (isTimeoutExceeded(time)) {
            setFinishReason(time + " seconds has been exceeded");
            return true;
        }

        return false;

    }

    /*
     * FIXME incorrect method signature
     * The correct method signature is
     * public void end(boolean interrupted)
     * 
     * See TimedDriveCommand
     * 
     */
    @Override
    public void end(boolean interrupted) {
        // FIXME log the command end
        // logCommandEnd(interrupted)


        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }

        logCommandEnd(interrupted);
    }

}
