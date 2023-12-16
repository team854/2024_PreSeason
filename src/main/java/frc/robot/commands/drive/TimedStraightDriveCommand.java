package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedStraightDriveCommand extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         time, leftSpeed, rightSpeed;
    private boolean        brakeAtEnd;

    private long           initialTime, currentTime;
    private long           runTime     = 0;

    /*
     * FIXME: make a constant HEADING_PID_KP in DriveConstants
     * 
     * The proportional gain value translates an error in degrees (-180 to +180)
     * to a motor speed adjustment value (-1.0 to +1.0)
     * The units will be %motorOuput/degree. Something in the range of .01 to .05 might be more
     * appropriate.
     */
    private double         K_p         = 1;
    private double         K_i         = 0;
    private double         K_d         = 0;

    private double         errorSignal = 0;
    private double         pTerm;
    private double         iTerm;
    private double         dTerm;


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
    public TimedStraightDriveCommand(double time, double leftSpeed, double rightSpeed, DriveSubsystem driveSubsystem) {
        this.time           = time;
        this.leftSpeed      = leftSpeed;
        this.rightSpeed     = rightSpeed;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        /*
         * RM:
         * FIXME Always log the command start
         * logCommandStart(parms)
         */
        initialTime = System.currentTimeMillis();

        // FIXME should the variable errorSignal be called initialHeading?
        errorSignal = driveSubsystem.getYaw();

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
        double currentError = driveSubsystem.getYaw();
        double diffError    = currentError - errorSignal;

        /*
         * FIXME start with a simple proportional gain
         * 
         * HINT: Start with just a single, simple proportional gain - this should be sufficient for
         * tracking a compass heading
         */
        pTerm        = K_p * errorSignal;
        iTerm       += K_i * errorSignal;
        dTerm       += K_d * diffError;

        // FIXME error should be a local variable
        // this code below modifies the initial heading
        errorSignal  = pTerm + iTerm + dTerm;

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
        leftSpeed   -= errorSignal;
        rightSpeed  += errorSignal;

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

    }

    @Override
    public boolean isFinished() {

        // FIXME isFinished()
        // should return true or false - the brakeAtEnd should be passed into the command, see
        // TimedDrive command
        currentTime = System.currentTimeMillis();
        runTime     = currentTime - initialTime;
        brakeAtEnd  = false;

        if (runTime <= time) {
            brakeAtEnd = false;
        }
        else {
            brakeAtEnd = true;
        }

        return brakeAtEnd;

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
    public void end(boolean brakeAtEnd) {
        // FIXME log the command end
        // logCommandEnd(interrupted)

        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }
    }

}
