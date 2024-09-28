package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.operator.GameController;

public class LightsSubsystem extends SubsystemBase {

   // Define lights
   private final AddressableLED              ledStrip             = new AddressableLED(LightConstants.LED_STRIP_PWM_PORT);
   private final AddressableLEDBuffer        ledBuffer            = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);

   private static final AddressableLEDBuffer RSL_OFF              = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);
   private static final AddressableLEDBuffer RSL_ON               = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);

   // Colors
   private static final Color                RSL_COLOR            = new Color(250, 21, 0);
   private static final Color                BOOST_COLOR          = new Color(0, 0, 127);
   private static final Color                TANK_COLOR           = new Color(0, 255, 0);
   private static final Color                DUAL_STICK_COLOR     = new Color(0, 255, 255);
   private static final Color                SINGLE_STICK_COLOR   = new Color(255, 255, 0);
   private static final Color                NON_BOOST_COLOR      = new Color(0, 0, 255);
   private static final Color                NO_SPEED_COLOR       = new Color(255, 255, 255);
   private static final Color                NOTHING_COLOR        = new Color(0, 0, 0);
   private static final Color                INTAKING_COLOR       = new Color(255, 140, 0);
   private static final Color                NOTE_INTOOK_COLOR    = new Color(255, 255, 255);
   private static final Color                POSESSION_COLOR      = new Color(0, 255, 0);

   // General Variables
   private static final int                  TRAIL_LENGTH         = 5;
   private boolean                           driveVisualization   = false;
   private int                               rslFlashCount        = -1;
   private boolean                           prevRSLOn            = false;
   private int                               rainbowFirstPixelHue = 0;

   // Variables for the Knight Rider effect
   private int                               knightFrontIndex     = 0;
   private int                               knightBackIndex      = 29;
   private boolean                           knightFrontDirection = true;
   private boolean                           knightBackDirection  = false;

   // Variables for countdown effect
   private boolean                           countdownActive      = false;
   private double                            lastMatchTime        = -1;

   // Game controller
   public final GameController               driverController     = new GameController(
      OperatorConstants.DRIVER_CONTROLLER_PORT,
      OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);

   static {
      // Initialize the RSL buffers
      for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
         RSL_ON.setLED(i, new Color(250, 21, 0));
         RSL_OFF.setLED(i, Color.kBlack);
      }
   }

   public LightsSubsystem() {
      // Start with all LEDs at a medium grey (not too blinding)
      ledStrip.setLength(LightConstants.LED_STRIP_LENGTH);
      setAllLEDs(Color.kBlack); // Start with all LEDs off
      ledStrip.start();

      // Add the camera here
      CameraServer.startAutomaticCapture();
   }

   public void setEnabled() {
      // When the robot is first enabled, flash the LEDs with the robot RSL for 5 flashes
      rslFlashCount = 5;
   }

   @Override
   public void periodic() {
      // Flash all lights in time with the RSL when the robot is first enabled
      if (rslFlashCount >= 0) {
         flashRSL();
      }

      // Check the match time and control LED behavior accordingly
      double matchTime = DriverStation.getMatchTime();
      if (matchTime <= 20 && matchTime != -1) {
         runCountdownEffect(matchTime); // Turn off LEDs progressively in the last 20 seconds
      }
      else if (!DriverStation.isEnabled()) {
         runKnightRiderEffect(); // Knight Rider effect when the robot is disabled
      }
      else {
         setAllianceColorOrRainbow(); // Default alliance color or rainbow effect when enabled
      }

      // After match ends, reset LEDs
      if (DriverStation.isDisabled() && countdownActive) {
         resetLEDsAfterMatch(); // Reset all LEDs after match ends
         countdownActive = false;
      }

      // Ensure the data is always sent to the strip, even if not flashing
      ledStrip.setData(ledBuffer);
   }

   // Method to run the LED countdown effect during the last 20 seconds
   public void runCountdownEffect(double matchTime) {
      countdownActive = true;

      // Number of LEDs to turn off based on the time left (20 seconds -> 60 LEDs)
      int ledsOff = (int) (60 * ((20.0 - matchTime) / 20.0));

      // Turn off LEDs from the outside towards the center
      for (int i = 0; i < ledsOff / 2; i++) {
         ledBuffer.setLED(i, NOTHING_COLOR); // Turn off front section
         ledBuffer.setLED(59 - i, NOTHING_COLOR); // Turn off back section
      }

      ledStrip.setData(ledBuffer);
   }

   // Method to reset all LEDs after the match ends
   public void resetLEDsAfterMatch() {
      setAllLEDs(Color.kWhite); // Turn all LEDs back on to white after match ends
   }

   // Method to run the Knight Rider effect with a fading trail
   // Front section moves left to right, Back section moves right to left
   public void runKnightRiderEffect() {
      // Set all LEDs to the background color first (black/off)
      setAllLEDs(Color.kBlack);

      // Draw the trail for the front section (LEDs 0-29)
      drawKnightRiderTrail(knightFrontIndex, 0, knightFrontDirection);

      // Draw the trail for the back section (LEDs 30-59)
      drawKnightRiderTrail(knightBackIndex, 30, knightBackDirection);

      // Update the knightFrontIndex for the front section movement
      if (knightFrontDirection) {
         knightFrontIndex++;
         if (knightFrontIndex >= 29) {
            knightFrontDirection = false; // Change direction at the end of the front section
         }
      }
      else {
         knightFrontIndex--;
         if (knightFrontIndex <= 0) {
            knightFrontDirection = true; // Change direction at the beginning of the front section
         }
      }

      // Update the knightBackIndex for the back section movement
      if (knightBackDirection) {
         knightBackIndex++;
         if (knightBackIndex >= 29) {
            knightBackDirection = false; // Change direction at the end of the back section
         }
      }
      else {
         knightBackIndex--;
         if (knightBackIndex <= 0) {
            knightBackDirection = true; // Change direction at the beginning of the back section
         }
      }

      ledStrip.setData(ledBuffer);
   }

   // Draw the Knight Rider trail on the LED strip
   // index is the current "main" LED position, offset is the starting position for the section
   // direction controls whether the LEDs move forward or backward
   private void drawKnightRiderTrail(int index, int offset, boolean direction) {
      for (int i = 0; i < TRAIL_LENGTH; i++) {
         int ledIndex = direction ? index - i : index + i; // Adjust index for trail
         if (ledIndex >= 0 && ledIndex < 30) { // Only set LEDs within the section
            int brightness = 255 - (i * (255 / TRAIL_LENGTH)); // Decrease brightness for the trail
            ledBuffer.setLED(offset + ledIndex, Color.fromHSV(0, 255, brightness)); // Red hue with
                                                                                    // fading
                                                                                    // brightness
         }
      }
   }

   // Method to turn LEDs orange while intaking
   public void setIntakingColor() {
      setAllLEDs(INTAKING_COLOR);
   }

   // Method to flash white for a note intake
   public void flashWhite() {
      setAllLEDs(NOTE_INTOOK_COLOR);
   }

   // Method to set LEDs green when the robot is holding a note
   public void setPossessionColor() {
      setAllLEDs(POSESSION_COLOR);
   }

   // Method to run a rainbow wave effect
   public void runRainbow() {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
         // Calculate the hue for this pixel. The hue is offset by the pixel's position
         int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
         ledBuffer.setLED(i, Color.fromHSV(hue, 255, 128));
      }

      // Increment the first pixel hue to create the wave effect
      rainbowFirstPixelHue += 3;
      rainbowFirstPixelHue %= 180; // Reset after a full cycle
      ledStrip.setData(ledBuffer);
   }

   // Set lights based on the alliance color or rainbow if no alliance
   public void setAllianceColorOrRainbow() {
      Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();

      if (allianceOptional.isPresent()) {
         DriverStation.Alliance alliance = allianceOptional.get();
         if (alliance == DriverStation.Alliance.Red) {
            setAllLEDs(Color.kRed);
         }
         else if (alliance == DriverStation.Alliance.Blue) {
            setAllLEDs(Color.kBlue);
         }
         else {
            runRainbow();
         }
      }
      else {
         runRainbow();
      }
   }

   // Method to turn off lights
   public void turnOffLights() {
      setAllLEDs(NOTHING_COLOR);
   }

   // Utility to set all LEDs to the same color
   private void setAllLEDs(Color color) {
      for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
         ledBuffer.setLED(i, color);
      }
      ledStrip.setData(ledBuffer);
   }

   private void flashRSL() {
      // Flash in time with the RSL light
      boolean rslOn = RobotController.getRSLState();

      // When the RSL changes from on to off, then update the RSL count
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
      if (!driveVisualization) {
         return; // Skip if drive visualization is disabled
      }

      int    leftMidPoint  = 15;
      int    rightMidPoint = 45;

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
         break;

      case DUAL_STICK_ARCADE:
      default:
         upToLeft = (int) Math.round(leftMidPoint - (leftStickY * LightConstants.LED_STICK_TAKEN_LENGTH));
         upToRight = (int) Math.round(rightMidPoint + (rightStickX * LightConstants.LED_STICK_TAKEN_LENGTH));

         for (int i = leftMidPoint - LightConstants.LED_STICK_TAKEN_LENGTH + 1; i < leftMidPoint
            + LightConstants.LED_STICK_TAKEN_LENGTH; i++) {
            ledBuffer.setLED(i, NOTHING_COLOR);
         }

         int adder = leftMidPoint - upToLeft;
         if (adder != 0) {
            for (int i = leftMidPoint; i != upToLeft; i = i - (adder / Math.abs(adder))) {
               ledBuffer.setLED(i, DUAL_STICK_COLOR);
            }
         }

         for (int i = rightMidPoint - LightConstants.LED_STICK_TAKEN_LENGTH + 1; i < rightMidPoint
            + LightConstants.LED_STICK_TAKEN_LENGTH; i++) {
            ledBuffer.setLED(i - 1, NOTHING_COLOR);
         }

         adder = rightMidPoint - upToRight;
         if (adder != 0) {
            for (int i = rightMidPoint; i != upToRight; i = i - (adder / Math.abs(adder))) {
               ledBuffer.setLED(i - 1, DUAL_STICK_COLOR);
            }
         }

         if (boost) {
            ledBuffer.setLED(LightConstants.BOOST_INDEX, BOOST_COLOR);
         }
         else {
            ledBuffer.setLED(LightConstants.BOOST_INDEX, NOTHING_COLOR);
         }
         break;
      }

      ledStrip.setData(ledBuffer);
   }
}
