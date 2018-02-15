package ca.mcgill.ecse211.project;


import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;

/**
 * This class provides utility methods to search for and identify the flag
 * 
 * @author Karl Godin, Victor Vuong, Evan Laflamme
 *
 */

public class FlagCapture {
  /** Sensor object in front of the robot */
  private EV3ColorSensor frontLightSensor = new EV3ColorSensor(SensorPort.S4);
  /** Odometer */
  private Odometer odometer;
  /** Navigator */
  private Navigator nav;
  /** Wifi Game Parameters */
  private GameParameter gameParams;


  public FlagCapture(Odometer odometer, Navigator nav, WallDetector wallDetector,
      GameParameter gameParams) {
    this.odometer = odometer;
    this.nav = nav;
    this.gameParams = gameParams;
  }


  /**
   * Begins searching for the flag in a given area by rotating a certain amount of degrees spanning
   * the area
   * 
   * @param x the x parameter is the current x position of the robot
   * @param y the y parameter is the current y position of the robot
   */
  public void searchFlag(int x, int y) {
    boolean capture = false;
    int position = 0;

    // x and y are the current corner (aka first corner we are at when starting the flag search)
    int corner[] = computeCorners(x, y);
    corner[0] = x;
    corner[1] = y;

    while (!capture) {
      nav.setMotorSpeeds(100);

      // turn towards next corner and then sweep clockwise
      double differenceX = corner[(position + 2) % 8] - corner[position % 8];
      double differenceY = corner[(position + 3) % 8] - corner[(position + 1) % 8];
      nav.turnTo(Math.atan2(differenceY, differenceX));

      // rotate clockwise
      nav.rotate(true, Math.PI / 2, 50);

      int sample = WallDetector.usPoller.fetchSample();

      int smallestSample = sample;
      double smallestAngle = 0;

      // stop sweeping and rotating if object detected
      while (nav.getLeftMotor().isMoving()) { // do nothing if
                                              // bigger
        if (sample < smallestSample) {
          smallestSample = sample;
          smallestAngle = odometer.getTheta();
        }

        sample = WallDetector.usPoller.fetchSample();
        // System.out.println(Math.toDegrees(odometer.getTheta()) + "\t" + sample);
      }
      // System.out.println("Next");
      nav.turnTo(smallestAngle);
      // move towards the block until color checking range
      nav.moveForward(100);

      double initialX = odometer.getX();
      double initialY = odometer.getY();
      double distX = 0;
      double distY = 0;
      double distanceTravelled = 0;

      int color = -1;

      frontLightSensor.setCurrentMode("RGB");
      float[] data = new float[3];
      frontLightSensor.fetchSample(data, 0);

      while (color == -1) {
        wait(100);

        frontLightSensor.fetchSample(data, 0);

        color = getColorFromRGB(data);

        distX = odometer.getX() - initialX;
        distY = odometer.getY() - initialY;

        distanceTravelled = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));

        if (distanceTravelled > (DPMProject.TILE_LENGTH * 3 / 2)) {
          break;
        }
      }
      nav.stopMotors();

      // checking color of the object
      if (color == DPMProject.flagColor) {
        captureFlag();

        nav.setMotorSpeeds(200);
        nav.driveDist(-distanceTravelled);

        return;
      } else {
        nav.setMotorSpeeds(200);

        nav.driveDist(-distanceTravelled);

        position = (position + 2) % 8;
        while (corner[position] == 0 || corner[position] == DPMProject.BOARD_LENGTH
            || corner[position + 1] == 0 || corner[position + 1] == DPMProject.BOARD_LENGTH) { // Ensure
                                                                                               // that
                                                                                               // robot
                                                                                               // does
                                                                                               // not
                                                                                               // travel
                                                                                               // into
                                                                                               // a
                                                                                               // wall
                                                                                               // corner
          position = (position + 2) % 8;
        }

        if (position == 0) { // If tried every corner, accept defeat
          acceptDefeat();
          break;
        }

        // travel to the next corner
        nav.travelTo(corner[position], corner[position + 1]);
      }
    }
  }

  /**
   * This method will read the values taken in from the RBG color sensor and manipulates it to find
   * relative values and return the most probable color detected
   * 
   * @param data data is the array containing the three values of R G B detected from the color
   *        sensor
   * @return the most probable color detected in int
   * 
   */
  public static int getColorFromRGB(float[] data) {
    float r = data[0];
    float g = data[1];
    float b = data[2];

    // Total color intensities
    float sumOfIntensities = r + g + b;

    // Relative intenisities
    float relativeR = r / sumOfIntensities;
    float relativeG = g / sumOfIntensities;
    float relativeB = b / sumOfIntensities;

    // Thresholds for the colors
    float threshold = 0.10f;
    float thresholdY = 0.06f;
    float thresholdW = 0.10f;

    // Based on tests, each block color corresponds to a relative amount of red, green, blue
    if (relativeR > (0.8 - threshold) && relativeR < (0.8 + threshold)
        && relativeG > (0.1 - threshold) && relativeG < (0.1 + threshold)
        && relativeB > (0.05 - threshold) && relativeB < (0.05 + threshold)) { // Case red
      return Color.RED;
    } else if (relativeR > (0.55 - thresholdY) && relativeR < (0.55 + thresholdY)
        && relativeG > (0.4 - thresholdY) && relativeG < (0.4 + thresholdY)
        && relativeB > (0.05 - thresholdY) && relativeB < (0.05 + thresholdY)) { // Case yellow
      return Color.YELLOW;
    } else if (relativeR > (0.15 - threshold) && relativeR < (0.15 + threshold)
        && relativeG > (0.4 - threshold) && relativeG < (0.4 + threshold)
        && relativeB > (0.43 - threshold) && relativeB < (0.43 + threshold)) { // Case blue
      return Color.BLUE;
    } else if (relativeR > (0.37 - thresholdW) && relativeR < (0.37 + thresholdW)
        && relativeG > (0.4 - thresholdW) && relativeG < (0.4 + thresholdW)
        && relativeB > (0.225 - thresholdW) && relativeB < (0.225 + thresholdW)) { // Case white
      return Color.WHITE;
    } else {
      return -1;
    }
  }

  /**
   * Makes the robot beep three times when the flag has been successfully identified
   */
  public void captureFlag() {
    Sound.beep(); // PO
    Sound.beep(); // LO
    Sound.beep(); // ??
  }

  /**
   * Makes the robot beep five times in order to indicate the flag was not found
   */
  public void acceptDefeat() {
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();
  }

  /**
   * This method will take in the current corner the robot is at along with the other points sent
   * from the server to compute the 4 corners of the search area
   * 
   * @param x x is the starting corner x position where the robot will start searching
   * @param y y is the starting corner y position where the robot will start searching
   */
  private int[] computeCorners(int x, int y) {
    int[] corners = new int[8];

    int xLL = 0;
    int yLL = 0;
    int xUR = 0;
    int yUR = 0;

    if (!DPMProject.isGreen) {
      xLL = gameParams.SG_LL_x;
      yLL = gameParams.SG_LL_y;
      xUR = gameParams.SG_UR_x;
      yUR = gameParams.SG_UR_y;
    } else {
      xLL = gameParams.SR_LL_x;
      yLL = gameParams.SR_LL_y;
      xUR = gameParams.SR_UR_x;
      yUR = gameParams.SR_UR_y;
    }

    if (xLL == x && yLL == y) {
      // Lower Left Corner
      corners[0] = xLL;
      corners[1] = yLL;
      corners[2] = xLL;
      corners[3] = yUR;
      corners[4] = xUR;
      corners[5] = yUR;
      corners[6] = xUR;
      corners[7] = yLL;

    } else if (xUR == x && yLL == y) {
      // Lower Right Corner
      corners[6] = xUR;
      corners[7] = yUR;
      corners[0] = xUR;
      corners[1] = yLL;
      corners[2] = xLL;
      corners[3] = yLL;
      corners[4] = xLL;
      corners[5] = yUR;
    } else if (xUR == x && yUR == y) {
      // Upper Right Corner
      corners[4] = xLL;
      corners[5] = yLL;
      corners[6] = xLL;
      corners[7] = yUR;
      corners[0] = xUR;
      corners[1] = yUR;
      corners[2] = xUR;
      corners[3] = yLL;
    } else if (xLL == x && yUR == y) {
      // Upper Left Corner
      corners[2] = xUR;
      corners[3] = yUR;
      corners[4] = xUR;
      corners[5] = yLL;
      corners[6] = xLL;
      corners[7] = yLL;
      corners[0] = xLL;
      corners[1] = yUR;
    } else {
      System.out.println("ERROR: Points do not match any corners");
    }

    return corners;
  }

  /**
   * This method will put the robot to sleep for a desired amount of time
   * 
   * @param time is the time desired in milliseconds
   */
  public static void wait(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
