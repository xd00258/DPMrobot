package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;

/**
 * This class is used to localize the robot using light sensor to detect grid lines. The robot will
 * know where it is relative to the corner surrounded by the four grid lines.
 * 
 * @author Victor Vuong
 *
 */
public class LightLocalizer {

  // system car specs
  private Odometer odometer;
  public LineDetector detector = new LineDetector("S2");
  private Navigator nav;

  // variables used for localizing
  private double sensorPositionX = 12.5;
  private double sensorPositionY = 12.5;

  // LightLocalizer constructor
  public LightLocalizer(Odometer odometer, UltrasonicPoller usPoller, Navigator navigator) {
    this.odometer = odometer;
    this.nav = navigator;
  }


  /**
   * This is the main method that localizes the robot using grid line intersections. The robot will
   * know its relative position from the intersection formed by the four grid lines detected.
   */
  public void initialLocalize() {
    double nearestIntersectX;
    double nearestIntersectY;
    double angleCorrectCorner = 0;

    double sensorPositionX = this.sensorPositionX;
    double sensorPositionY = this.sensorPositionY;

    // the nearest intersect depends on the starting corner
    nearestIntersectX =
        DPMProject.TILE_LENGTH * PathGenerator.getStartX(DPMProject.startingPosition);
    nearestIntersectY =
        DPMProject.TILE_LENGTH * PathGenerator.getStartY(DPMProject.startingPosition);

    if (DPMProject.startingPosition == 0) {
      angleCorrectCorner = 0;
      sensorPositionX = -sensorPositionX;
      sensorPositionY = -sensorPositionY;
    } else if (DPMProject.startingPosition == 1) {
      angleCorrectCorner = 90;
      sensorPositionX = -sensorPositionX;
    } else if (DPMProject.startingPosition == 2) {
      angleCorrectCorner = 180;
    } else if (DPMProject.startingPosition == 3) {
      angleCorrectCorner = 270;
      sensorPositionY = -sensorPositionY;
    }

    int counter = 0; // counter variable to keep track of how many lines detected
    double[] angle = new double[5]; // array to store the 4 angles saved from light localizing
    nav.rotate(true, 100);; // rotation speed of wheels

    // Determines the 5 angles at which a line is detected
    while (counter < 5) {
      if (detector.hasLine()) {
        // Get the current angle
        double[] pos = new double[3];
        odometer.getPosition(pos, new boolean[] {true, true, true});
        angle[counter] = pos[2];

        // Indicate a line was detected
        Sound.beep();

        counter++;
      }
    }

    // stop rotating
    nav.stopMotors();

    // Since the sensor is in the back of the robot, the orientation must be flipped by 180 degrees
    angle[0] -= 180;
    angle[1] -= 180;
    angle[2] -= 180;
    angle[3] -= 180;
    angle[4] -= 180;

    // Computing the angle X and angle Y
    double thetaY = angle[3] - angle[1];
    double thetaX = angle[2] - angle[0];


    // Correct the track value
    double differenceForTrackValue = angle[4] - angle[0];
    DPMProject.TRACK = DPMProject.TRACK / ((differenceForTrackValue / 360) + 1);

    thetaY = Math.toRadians(thetaY);
    thetaX = Math.toRadians(thetaX);

    double x;
    double y;

    // Proper correction based on starting corner
    if (DPMProject.startingPosition == 0 || DPMProject.startingPosition == 2) {
      x = sensorPositionX * Math.cos(thetaY / 2) + nearestIntersectX;
      y = sensorPositionY * Math.cos(thetaX / 2) + nearestIntersectY;
    } else {
      x = sensorPositionY * Math.cos(thetaX / 2) + nearestIntersectX;
      y = sensorPositionX * Math.cos(thetaY / 2) + nearestIntersectY;
    }

    // Calculate correct angle
    double dTheta = 180 - (angle[0] + angle[2]) / 2;
    double theta = Math.toDegrees(odometer.getTheta()) + dTheta;

    // Set the corrected angle
    odometer.setPosition(
        new double[] {x, y,
            (theta) * Math.PI / 180 - Math.PI / 2 + Math.toRadians(angleCorrectCorner)},
        new boolean[] {true, true, true});

    try {
      Thread.sleep(1500);
    } catch (InterruptedException ex) {
      // none
    }
  }

  /**
   * Performs localization without the assumption that the robot is in a starting corner at the
   * closest intersection.
   * 
   * @param reverse states whether or not the robot should go in reverse
   */
  public void localize(boolean reverse) {
    double cX = Math.round(odometer.getX() / 30); // getting closest intersection
    double cY = Math.round(odometer.getY() / 30);

    localize(cX, cY, reverse);
  }

  /**
   * Performs localization without the assumption that the robot is in a starting corner.
   * 
   * @param closestX its closest intersection in X units
   * @param closestY its closest intersection in Y units
   * @param reverse states whether or not the robot should go in reverse
   */
  public void localize(double closestX, double closestY, boolean reverse) {

    // Turn to 45 degrees and move forward until it sees a line. This is to make sure the robot is
    // close enough to an intersection
    nav.turnTo(Math.PI / 4);
    nav.moveForward();
    while (true) {
      if (detector.hasLine()) {
        nav.stopMotors();

        if (reverse)
          nav.setMotorSpeeds(DPMProject.BACKWARD_SPEED);
        nav.driveDist(-20); // 18
        nav.setMotorSpeeds(DPMProject.FORWARD_SPEED);

        break;
      }
    }

    // Nearest position in cm
    double nearestIntersectX = closestX * DPMProject.TILE_LENGTH;
    double nearestIntersectY = closestY * DPMProject.TILE_LENGTH;

    // Corner in which the robot is.
    double angleCorrectCorner = 0;

    // Position of the sensor
    double sensorPositionX = -12.5;
    double sensorPositionY = -12.5;


    int counter = 0; // counter variable to keep track of how many lines detected
    double[] angle = new double[4]; // array to store the 4 angles saved from light localizing

    nav.rotate(true, 100); // rotation speed of wheels

    // Determines the 4 angles at which a line is detected
    while (counter < 4) {
      if (detector.hasLine()) {
        // Get the current angle
        double[] pos = new double[3];
        odometer.getPosition(pos, new boolean[] {true, true, true});
        angle[counter] = pos[2];

        // Indicate whether or not a line had been detected with sound
        Sound.beep();

        counter++;
      }
    }

    // stop rotating
    nav.stopMotors();

    // since the sensor is in the back of the robot, the orientation must be flipped by 180 degrees
    angle[0] -= 180;
    angle[1] -= 180;
    angle[2] -= 180;
    angle[3] -= 180;

    // computing the angle X and angle Y
    double thetaY = angle[3] - angle[1];
    double thetaX = angle[2] - angle[0];
    thetaY = Math.toRadians(thetaY);
    thetaX = Math.toRadians(thetaX);

    // Angle difference between each angle
    double omega = angleDifference(angle[1], angle[0]);
    double alpha = angleDifference(angle[2], angle[1]);
    double beta = angleDifference(angle[3], angle[2]);
    double gamma = angleDifference(angle[0], angle[3]);

    // Finds the index of the largest angle
    int quadrant = findMaxAngle(omega, alpha, beta, gamma);

    // Corrects angle based on which quadrant the robot is in
    switch (quadrant) {
      case 0:
        angleCorrectCorner = 0;
        break;
      case 1:
        angleCorrectCorner = 0;
        break;
      case 2:
        sensorPositionY = -sensorPositionY;
        angleCorrectCorner = 180;
        break;
      case 3:
        sensorPositionY = -sensorPositionY;
        angleCorrectCorner = 180;
        break;
    }

    // Computing the real x and y position of the robot relative to the 0, 0
    double x = sensorPositionX * Math.cos(thetaY / 2) + nearestIntersectX;
    double y = sensorPositionY * Math.cos(thetaX / 2) + nearestIntersectY;

    // Compute the correct angle
    double dTheta = 180 - (angle[0] + angle[2]) / 2;
    double theta = Math.toDegrees(odometer.getTheta()) + dTheta;

    double newTheta = (theta) * Math.PI / 180 - Math.PI / 2 + Math.toRadians(angleCorrectCorner);

    // Wraps the angle between 0 and 360
    if (newTheta > 2 * Math.PI) {
      newTheta -= 2 * Math.PI;
    } else if (newTheta < 0) {
      newTheta += 2 * Math.PI;
    }

    // Set the corrected angle
    odometer.setPosition(new double[] {x, y, newTheta}, new boolean[] {true, true, true});

    try {
      Thread.sleep(1500);
    } catch (InterruptedException ex) {
    }
  }

  /**
   * Finds the index of the largest angle out of 4.
   * 
   * @param omega first angle
   * @param alpha second angle
   * @param beta third angle
   * @param gamma fourth angle
   * @return the index of the largest angle
   */
  private int findMaxAngle(double omega, double alpha, double beta, double gamma) {
    // Store angles into an array for ease of use
    double[] angles = new double[] {alpha, omega, gamma, beta};
    int max = 0; // Holds index of max element

    for (int i = 0; i < angles.length; i++) {
      if (angles[i] > angles[max]) { // If current element is greater than previous max
        max = i; // Set new max
      }
    }

    return max;
  }

  /**
   * Finds the smallest difference between two angles.
   * 
   * @param target The new angle
   * @param source The old angle
   * @return the difference between the source and target angles
   */
  private double angleDifference(double target, double source) {
    double diff = target - source;

    if (diff > 180) {
      diff -= 360;
    } else if (diff < -180) {
      diff += 360;
    }

    return diff;
  }

  /**
   * Performs the angle correction without the use of an intersection. This is done by rotating
   * clockwise until a line and then anti clockwise until a line. An error is then calculated by
   * determining the midpoint between these two angles
   */
  public void wiggleCorrection() {
    // Makes sure the light sensor is close enough to a line to perform correction
    nav.moveForward();
    while (true) {
      if (detector.hasLine()) {
        nav.stopMotors();
        nav.driveDist(-5);
        break;
      }
    }

    // The initial angle
    double initialAngle = odometer.getTheta();

    // Rotate clockwise until a line is detected
    nav.rotate(true, 100);
    while (!detector.hasLine()) {

    }
    nav.stopMotors();
    // Store first line angle
    double angle1 = odometer.getTheta();

    // Rotate anti clockwise until a line is detected
    nav.rotate(false, 100);
    try {
      Thread.sleep(500);
    } catch (InterruptedException ex) {
      // none
    }
    while (!detector.hasLine()) {

    }
    nav.stopMotors();

    // Calculate the angle correction
    double angle2 = ((odometer.getTheta() - angle1) + Math.PI * 2) % (Math.PI * 2);
    double deltaAngle = angle2 / 2 - (initialAngle - angle1);
    odometer.setTheta(odometer.getTheta() - deltaAngle);
  }
}
