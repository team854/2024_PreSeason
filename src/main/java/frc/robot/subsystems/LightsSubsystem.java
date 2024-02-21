
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.operator.GameController;


public class LightsSubsystem extends SubsystemBase {

   // define lights
   private final AddressableLED              ledStrip           = new AddressableLED(LightConstants.LED_STRIP_PWM_PORT);
   private final AddressableLEDBuffer        ledBuffer          = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);

   private static final AddressableLEDBuffer RSL_OFF            = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);
   private static final AddressableLEDBuffer RSL_ON             = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);

   // Colours
   private static final Color                RSL_COLOR          = new Color(250, 21, 0);
   private static final Color                BOOST_COLOR        = new Color(0, 0, 127);
   private static final Color                TANK_COLOR         = new Color(0, 255, 0);
   private static final Color                DUAL_STICK_COLOR   = new Color(0, 255, 255);
   private static final Color                SINGLE_STICK_COLOR = new Color(255, 255, 0);
   private static final Color                NON_BOOST_COLOR    = new Color(0, 0, 255);
   private static final Color                NO_SPEED_COLOR     = new Color(255, 255, 255);
   private static final Color                NOTHING_COLOR      = new Color(0, 0, 0);


   private int                               rslFlashCount      = -1;
   private boolean                           prevRSLOn          = false;

   // game controller
   public final GameController               driverController   = new GameController(
      OperatorConstants.DRIVER_CONTROLLER_PORT,
      OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);

   static {
      // Initialize the RSL buffers
      for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
         RSL_ON.setLED(i, RSL_COLOR);
         RSL_OFF.setLED(i, Color.kBlack);
      }
   }

   public LightsSubsystem() {

      // Start with all LEDs at a medium grey (not too blinding)
      ledStrip.setLength(LightConstants.LED_STRIP_LENGTH);
      setAllLEDs(new Color(250, 21, 0));
      ledStrip.start();
   }

   public void setEnabled() {

      // When the robot is first enabled flash the LEDs with the robot RSL for 5 flashes
      rslFlashCount = 5;
   }

   @Override
   public void periodic() {

      // Flash all lights in time with the RLS when the robot is first enabled
      if (rslFlashCount >= 0) {
         flashRSL();
      }
      else {
         ledStrip.setData(ledBuffer);
      }
   }

   private void clear() {
      for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
         ledBuffer.setLED(i, RSL_COLOR);
      }
   }

   private void setAllLEDs(Color color) {

      for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
         ledBuffer.setLED(i, color);
      }
   }

   private void setLED(int index, Color color) {
      ledBuffer.setLED(index, color);
   }

   private void flashRSL() {

      // Flash in time with the RSL light
      boolean rslOn = RobotController.getRSLState();

      // when the RSL changes from on to off, then
      // update the RSL count.
      if (prevRSLOn && !rslOn) {
         rslFlashCount--;
      }
      prevRSLOn = rslOn;

      // Set the LEDs based on the RSL state
      if (rslOn) {
         ledStrip.setData(RSL_ON);
      }
      else {
         ledStrip.setData(RSL_OFF);
      }
   }


   public void ledStick(boolean boost, DriveMode driveMode) {


      int    leftMidPoint  = (int) 15;
      int    rightMidPoint = (int) 45;

      double leftStickY    = driverController.getLeftY();
      double leftStickX    = driverController.getLeftX();
      double rightStickY   = driverController.getRightY();
      double rightStickX   = driverController.getRightX();

      int    upToLeft;
      int    upToRight;
      switch (driveMode) {

      case SINGLE_STICK_ARCADE:
         upToLeft = (int) Math.round(leftMidPoint * (1 - leftStickY));
         upToRight = (int) Math.round(rightMidPoint + leftMidPoint * leftStickX);

         for (int i = Math.min(upToLeft, leftMidPoint); i == Math.max(upToLeft, leftMidPoint); i++) {
            ledBuffer.setLED(i, SINGLE_STICK_COLOR);
         }


         if (boost) {
            ledBuffer.setLED(LightConstants.BOOST_INDEX, BOOST_COLOR);
         }
         else {
            ledBuffer.setLED(LightConstants.BOOST_INDEX, NOTHING_COLOR);
         }

      case DUAL_STICK_ARCADE:
      default:


         upToLeft = (int) Math.round(leftMidPoint - (leftStickY * LightConstants.LED_STICK_TAKEN_LENGTH));
         upToRight = (int) Math.round(rightMidPoint + (rightStickX * LightConstants.LED_STICK_TAKEN_LENGTH));
         // System.out.println(upToLeft);

         for (int i = (leftMidPoint - LightConstants.LED_STICK_TAKEN_LENGTH + 1); i < leftMidPoint
            + LightConstants.LED_STICK_TAKEN_LENGTH; i++) {
            ledBuffer.setLED(i, NOTHING_COLOR);
         }
         int Adder;
         Adder = (leftMidPoint - upToLeft);
         if (Adder != 0) {
            for (int i = leftMidPoint; i != upToLeft; i = i - (Adder / Math.abs(Adder))) {
               // System.out.println(i);
               ledBuffer.setLED(i, DUAL_STICK_COLOR);
            }
         }

         for (int i = (rightMidPoint - LightConstants.LED_STICK_TAKEN_LENGTH + 1); i < rightMidPoint
            + LightConstants.LED_STICK_TAKEN_LENGTH; i++) {
            ledBuffer.setLED(i - 1, NOTHING_COLOR);
         }

         Adder = (rightMidPoint - upToRight);
         if (Adder != 0) {
            for (int i = rightMidPoint; i != upToRight; i = i - (Adder / Math.abs(Adder))) {
               // System.out.println(i);
               ledBuffer.setLED(i - 1, DUAL_STICK_COLOR);
            }
         }

         if (boost) {
            ledBuffer.setLED(LightConstants.BOOST_INDEX, BOOST_COLOR);
         }
         else {
            ledBuffer.setLED(LightConstants.BOOST_INDEX, NOTHING_COLOR);
         }

      case TANK:


      }
   }


}

