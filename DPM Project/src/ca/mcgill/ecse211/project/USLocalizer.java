package ca.mcgill.ecse211.project;

/**
 * This class is used to localize the robot at the beginning of the competition using the Ultrasonic
 * Sensor
 * 
 * @author Victor Vuong
 *
 */
public class USLocalizer {

  /** Rotation speed of the robot */
  public static int ROTATION_SPEED = 150;
  /** Odometer of the robot */
  private Odometer odometer;
  /** Wall detector */
  private WallDetector wallDetector;
  /** Navigator */
  private Navigator navigator;

  // Adjustment variables for the localizer
  private double centerToUS = 11;
  private double AdjustmentX = 0;
  private double AdjustmentY = 0;
  private double angleAdjustment = 40;

  /**
   * US Localizer constructor
   * 
   * @param odometer Odometer
   * @param wallDetector Wall Detector
   * @param leftMotor Left Motor
   * @param rightMotor Right Motor
   */
  public USLocalizer(Odometer odometer, WallDetector wallDetector, Navigator nav) {
    this.odometer = odometer;
    this.wallDetector = wallDetector;
    this.navigator = nav;
  }


  /**
   * This is the main localizer method that uses advanced Differential calculus to detect walls
   * while rotating. The robot will find its approximate position relative to the first corner and
   * positions itself so that Light Localizer can be done
   */
  public void doLocalization() {
    // usPoller.start();
    int count = 0; // counter variable for the two minimums
    int currentDist; // current distance polled from US
    int nextDist; // next distance polled from US
    double[] minimum = new double[2]; // array to store the two minimums
    double[] angle = new double[2]; // array to store the two angles
    boolean negativeSlope; // boolean to know whether the robot is turning to a wall or away from
                           // wall

    navigator.rotate(false, ROTATION_SPEED);
    currentDist = getFilteredData();
    try {
      Thread.sleep(250);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    nextDist = getFilteredData();

    if (currentDist - nextDist > 0) { // checking if it is negative slope or not
      negativeSlope = true;
    } else {
      negativeSlope = false;
    }

    while (count < 2) { // getting the two minimums from the walls

      while (negativeSlope && count < 2) {
        try {
          Thread.sleep(250);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        nextDist = getFilteredData();

        if (nextDist - currentDist > 0) {
          minimum[count] = currentDist;
          angle[count] = Math.toDegrees(odometer.getTheta());
          negativeSlope = false;
          count++;
        }

        currentDist = nextDist;
      }

      while (!negativeSlope && count < 2) { // if not negative slope, turn until negative slope

        try {
          Thread.sleep(250);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }

        nextDist = getFilteredData();

        if (currentDist - nextDist > 0) {

          negativeSlope = true;
        }

        currentDist = nextDist;

      }
    }

    navigator.stopMotors();

    double angleDifference = (360 + (angle[1] - angle[0])) % 360; // compute the angles saved

    // setting the new position and angle of the robot relative to the corner
    if (angleDifference < 180) {

      double adjustedAngle = Math.toDegrees(odometer.getTheta()) + 270 - angle[1] + angleAdjustment;

      odometer.setX(minimum[0] - 30 + centerToUS + AdjustmentX);
      odometer.setY(minimum[1] - 30 + centerToUS + AdjustmentY);

      odometer.setTheta(adjustedAngle * Math.PI / 180);
    }

    if (angleDifference > 180) {
      double adjustedAngle = Math.toDegrees(odometer.getTheta()) + 180 - angle[1] + angleAdjustment;
      odometer.setX(minimum[1] - 30 + centerToUS + AdjustmentX);
      odometer.setY(minimum[0] - 30 + centerToUS + AdjustmentY);
      odometer.setTheta(adjustedAngle * Math.PI / 180);
    }

  }

  /**
   * This method will filter values polled from the ultrasonic sensor that are bigger than 60
   * 
   * @return distance polled from the ultrasonic sensor
   */
  public int getFilteredData() {
    int distance;

    if (wallDetector.getSample() > 60) {
      distance = 60;
    } else {
      distance = wallDetector.getSample();
    }

    return distance;
  }

}
