package ca.mcgill.ecse211.project;


import java.util.ArrayList;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.Color;

/**
 * The DPMProject class is the main executor class for the program. This class contains the main
 * method of the program. It is responsible for the main flow of the program. This class also
 * contains the global parameters that other classes will have to access.
 * 
 * The main method implements the main logic of the program. The robot firstly initializes its
 * subsystems including odometry, ultrasonic polling, etc. It then localizes to have an accurate
 * representation of its starting position and heading. Depending on which team the robot is on, it
 * then generates and travels along a path to the zipline or across the shallow region of the river.
 * Once the path completed, the robot relocalizes. It then generates a path toward the flag zone
 * depending on which zone it needs to search. Once at the opponent zone, the robot searches for the
 * flag and beep 3 times once found. It then relocalizes at its nearest point. It then generates a
 * path either across the river or across the zipline depending on its team. Once returned to its
 * home zone, the robot relocalizes, generates, and travels a path to its starting corner. Once at
 * its starting corner, it again relocalizes. The robot then stops, indicating the objective is
 * complete.
 *
 * Refer to the software document v5 for more details on implementation.
 * 
 * @author Evan Laflamme
 */

public class DPMProject {
  /** IP Address for game server */
  public static final String SERVER_IP = "192.168.2.3";
  /** Robot's team number */
  public static final int TEAM_NUMBER = 5;

  /** The left motor of the robot */
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  /** The right motor of the robot */
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  /** The arm motor to mount the zipline */
  private static final EV3LargeRegulatedMotor armMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  /** Radius of the robot's wheels */
  public static final double WHEEL_RADIUS = 2.07;
  /** Track distance between the robot's wheels */
  public static double TRACK = 11.5;
  /** Tile length of board */
  public static int BOARD_LENGTH = 12;
  /** Length of one tile */
  public static final double TILE_LENGTH = 30.48;
  /** Forward speed of the robot */
  public static int FORWARD_SPEED = 500;
  /** Backward speed of the robot */
  public static final int BACKWARD_SPEED = 200;
  /** Rotation speed of the robot */
  public static final int ROTATE_SPEED = 150;
  /** Correction to account for weight balancing with navigation */
  public static final double MOTOR_BALANCE = 1.009;

  /** The port for the ultrasonic sensor */
  private static final Port usPort = LocalEV3.get().getPort("S1");
  /** Ultrasonic poller */
  private static UltrasonicPoller usPoller = null;
  /** Wall detector */
  private static WallDetector wallDetector = null;
  /** The robot's odometer */
  private static Odometer odometer = null;
  /** The robot's odometry display */
  private static OdometryDisplay odometryDisplay = null;
  /** The robot's navigator */
  private static Navigator navigator = null;
  /** The robot's light localizer */
  public static LightLocalizer lightLocalizer = null;
  /** The robot's flag capture class */
  private static FlagCapture flagCapture = null;
  /** The robot's path generator */
  private static PathGenerator pathGenerator = null;
  /** The robot's LCD screen component */
  private static final TextLCD t = LocalEV3.get().getTextLCD();

  /** The robot's starting corner */
  public static volatile int startingPosition = 0;
  /** The robot's opponents flag color */
  public static volatile int flagColor;
  /** The robot's team: true if green, false if red */
  public static boolean isGreen = false;
  /** The robot's game parameters */
  private static GameParameter gameParams = null;


  /**
   * The main method implements the main logic of the program. The robot firstly initializes its
   * subsystems including odometry, ultrasonic polling, etc. It then localizes to have an accurate
   * representation of its starting position and heading. Depending on which team the robot is on,
   * it then generates and travels along a path to the zipline or across the shallow region of the
   * river. Once the path completed, the robot relocalizes. It then generates a path toward the flag
   * zone depending on which zone it needs to search. Once at the opponent zone, the robot searches
   * for the flag and beep 3 times once found. It then relocalizes at its nearest point. It then
   * generates a path either across the river or across the zipline depending on its team. Once
   * returned to its home zone, the robot relocalizes, generates, and travels a path to its starting
   * corner. Once at its starting corner, it again relocalizes. The robot then stops, indicating the
   * objective is complete.
   */
  public static void main(String[] args) {
    initializeSubsystems();

    // Set starting position based on current team
    if (gameParams.RedTeam == TEAM_NUMBER) {
      startingPosition = gameParams.RedCorner;
      isGreen = false;
    } else if (gameParams.GreenTeam == TEAM_NUMBER) {
      startingPosition = gameParams.GreenCorner;
      isGreen = true;
    } else {
      System.out.println("Error: Team number does not correspond to either team.");
      return;
    }

    // Set flag color
    int flag;

    if (isGreen) {
      flag = gameParams.OG;
    } else {
      flag = gameParams.OR;
    }

    // Map inputted flag color lejos colors
    switch (flag) {
      case 1:
        flagColor = Color.RED;
        break;
      case 2:
        flagColor = Color.BLUE;
        break;
      case 3:
        flagColor = Color.YELLOW;
        break;
      case 4:
        flagColor = Color.WHITE;
        break;
    }
    // Determine track value and approximate position
    performInitialLocalization();

    // Localize with appropriate track value for precise position
    lightLocalizer.localize(true);

    // Travel to starting position
    navigator.travelTo(PathGenerator.getStartX(startingPosition),
        PathGenerator.getStartY(startingPosition));

    // Generate appropriate path depending on team
    ArrayList<double[]> path = null;

    if (!isGreen) { // Starting by traversing shallow water
      path = pathGenerator.generateRiverPath();
    } else if (isGreen) { // Starting by traversing zipline
      path = pathGenerator.generateZiplinePath();
    }

    // Travel along generated path
    travelPath(path);

    // Relocalize once at final destination
    lightLocalizer.localize(true);

    // If green team, travel across the zipline
    if (isGreen) {
      navigator.travelTo(gameParams.ZO_G_x, gameParams.ZO_G_y);
      mountAndTraverseZipline();
    }

    // Travel to flag zone
    path = pathGenerator.generateFlagZonePath();
    travelPath(path);

    // Relocalize
    lightLocalizer.localize(true);

    // Travel to final position
    navigator.travelTo(path.get(path.size() - 1)[0], path.get(path.size() - 1)[1]);

    // Search for flag
    flagCapture.searchFlag(Math.round((float) (odometer.getX() / TILE_LENGTH)),
        Math.round((float) (odometer.getY() / TILE_LENGTH))); // Search for flag within flag zone

    // Generate appropriate path depending on team
    if (!isGreen) { // Starting by traversing shallow water
      path = pathGenerator.generateZiplinePath();
    } else if (isGreen) { // Starting by traversing zipline
      path = pathGenerator.generateRiverPath();
    }

    // Travel back to player zone
    travelPath(path);

    // If green team, relocalize and travel across the zipline
    if (!isGreen) {
      lightLocalizer.localize(true);
      mountAndTraverseZipline();
    }

    // Travel back to original position
    path = pathGenerator.generateHomePath();
    travelPath(path);
  }

  /**
   * This method initializes and activates the needed subsystems which include the ultrasonic
   * sensor, poller and controller, the odometer and its display, and the navigator.
   */
  private static void initializeSubsystems() {
    // Activate ultrasonic sensor, poller and controller
    wallDetector = new WallDetector(usPort);


    // Activate odometry subsystems
    odometer = new Odometer(leftMotor, rightMotor);
    odometryDisplay = new OdometryDisplay(odometer, t);

    odometer.start();
    odometryDisplay.start();

    // Activate navigator
    navigator = new Navigator(leftMotor, rightMotor, armMotor, odometer);

    // Activate light localizer
    lightLocalizer = new LightLocalizer(odometer, usPoller, navigator);

    // Get game parameters over wifi
    gameParams = new GameParameter();

    // Initialize the path generator class
    pathGenerator = new PathGenerator(gameParams, odometer);

    // Activate the flag capturer
    flagCapture = new FlagCapture(odometer, navigator, wallDetector, gameParams);
  }

  /**
   * This method performs the initial localization of the robot. It performs ultrasonic localization
   * to position itself approximately at the nearest point. It then performs light localization to
   * obtain a more accurate position.
   */
  private static void performInitialLocalization() {
    // Perform approximate ultrasonic localization
    USLocalizer usLocalizer = new USLocalizer(odometer, wallDetector, navigator);
    usLocalizer.doLocalization();

    // Travel to relative (0,0) and perform initial light localization
    navigator.travelTo(-0.1, -0.1);
    lightLocalizer.initialLocalize();

    // Travel to starting position
    navigator.travelTo(PathGenerator.getStartX(startingPosition),
        PathGenerator.getStartY(startingPosition));
  }

  /**
   * This method travels along the given path.
   * 
   * @param path an ArrayList holding the points along the path
   */
  private static void travelPath(ArrayList<double[]> path) {
    boolean wasLocalization = false;
    double[] previousPoint = null;

    // For every point in the path
    for (double[] point : path) {
      // If previously relocalized, go to the corrected position before going to next point
      if (wasLocalization) {
        navigator.travelTo(previousPoint[0], previousPoint[1]);
        wasLocalization = false;
      }
      navigator.travelTo(point[0], point[1]); // Travel to that point

      // If localization flag is set, localize first
      if (point[2] == 1) {
        wasLocalization = true;
        previousPoint = point;

        lightLocalizer.localize(true);
      } else if (point[2] == 2) { // If wiggle correction flag is set, perform wiggle correction
        double differenceX =
            (path.get(path.indexOf(point) + 1)[0] * DPMProject.TILE_LENGTH) - odometer.getX();
        double differenceY =
            (path.get(path.indexOf(point) + 1)[1] * DPMProject.TILE_LENGTH) - odometer.getY();

        navigator.turnTo(Math.atan2(differenceY, differenceX));

        lightLocalizer.wiggleCorrection();
      }

      wasLocalization = false;
    }
  }

  /**
   * This method uses the navigator method to mount and travel across the zipline. The robot then
   * relocalizes once it is at the closest point in order to reset its position.
   */
  private static void mountAndTraverseZipline() {
    boolean reverse = true;

    // Ensures that the robot's cone is always the first part hitting the zipline
    if (gameParams.ZO_G_x != gameParams.ZC_G_x && gameParams.ZO_G_y != gameParams.ZC_G_y) {
      reverse = false;
    }

    // Turn towards the zipline
    navigator.turnTo(
        Math.atan2(gameParams.ZC_G_y - gameParams.ZO_G_y, gameParams.ZC_G_x - gameParams.ZO_G_x)
            - Math.toRadians(5));

    navigator.mountAndTraverseZipline(!reverse);
    // Localize at the end
    lightLocalizer.localize(gameParams.ZO_R_x, gameParams.ZO_R_y, true);
    navigator.travelTo(gameParams.ZO_R_x, gameParams.ZO_R_y);
  }
}
