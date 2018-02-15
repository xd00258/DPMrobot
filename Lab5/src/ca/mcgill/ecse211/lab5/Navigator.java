package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.UltrasonicPoller;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigator extends Thread {

  private static final double NAVIGATOR_TOLERANCE = 1.5; // used to check if the robot arrived
                                                         // within tolerance distance

  private EV3LargeRegulatedMotor leftMotor;

  private EV3LargeRegulatedMotor rightMotor;

  private static final EV3LargeRegulatedMotor rotatingSensor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B")); // motor to rotate ultrasonic sensor

  private static final EV3LargeRegulatedMotor armWheel =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  private Odometer odometer;

  private UltrasonicPoller usPoller;

  private final double leftRadius = 2.1; // Specifications of the robot car
  private final double rightRadius = 2.1;
  private final double trackValue = Lab5.TRACK;
  private final int motorStraight = 200;
  private final float weightCompensation = 2.8F;

  // constructor for navigator
  public Navigator(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, UltrasonicPoller usPoller) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.usPoller = usPoller;
  }

  // Specifications for navigator
  private final double THETA_THRESHOLD = 0.05; // threshold angle for re orienting the robot
  private double[] path = new double[0]; // variable to contain the path waypoints
  private boolean isNavigating = false; // boolean to represent Navigating mode

  // Specifications for avoiding obstacles
  private double dangerRange = 15; // bandcenter of Avoiding Mode
  private boolean isAvoiding = false; // boolean to know if robot is in avoid mode
  private final int avoidAngle = -50; // angle of ultrasonic sensor when in avoid mode
  private final int navigatingAngle = 0; // angle of ultrasonic sensor when in navigating mode
  private int safeDist = 20; // distance before going into avoid mode
  private final int motorHigh = 300; // speed constant of bang bang controller
  private final int motorLow = 100;
  private final double orientationThresh = 0.025; // threshold angle to match the destination's
                                                  // angle


  public void travelToZip() {
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(500); // original is 3000
    }

    double prevX = 0;
    double prevY = 0;
    for (int i = 0; i < path.length; i += 2) {
      if (path[i] != prevX && path[i + 1] != prevY) {
        travelTo(path[i], path[i + 1]);
      }

      prevX = path[i];
      prevY = path[i + 1];
    }
  }

  // method to travel to a way point which also contains the avoid mode if an obstacle is present
  public void travelTo(double x, double y) // changed x and y
  {
    isNavigating = true; // entering navigating mode

    leftMotor.setSpeed(motorStraight);
    rightMotor.setSpeed(motorStraight);
    double[] pos = new double[3];
    double differenceX = x - odometer.getX(); // get the X distance between current position and the
                                              // destination
    double differenceY = y - odometer.getY(); // get the Y distance between current position and the
                                              // destination

    turnTo(Math.atan2(differenceY, differenceX)); // find the angle to reach the destination and
                                                  // turn to that angle

    leftMotor.forward(); // start moving forward;
    rightMotor.forward();


    // loop to check for obstacle and destination arrival
    while (true) {
      // collect data of the position and angle of the robot in case it is off track
      double currentTheta = odometer.getTheta();
      double theta = Math.atan2(differenceY, differenceX) * 180 / Math.PI;
      double absDTheta = Math.abs(theta - currentTheta);
      differenceX = x - odometer.getX();
      differenceY = y - odometer.getY();

      // correct if angle is off from target waypoint
      if (absDTheta < THETA_THRESHOLD) ////////
      {
        turnTo(Math.atan2(differenceY, differenceX));

        leftMotor.forward();
        rightMotor.forward();
      }



      // check if arrived at destination with the euclidean error
      differenceX = x - odometer.getX();
      differenceY = y - odometer.getY();

      // arrived, stop motor and end navigation and avoid mode
      if (differenceX * differenceX + differenceY * differenceY < NAVIGATOR_TOLERANCE) {
        leftMotor.stop(true);
        rightMotor.stop(false);
        Sound.beep();
        isNavigating = false;
        // turnTo(angle);
        return;
      }
    }
    /*
     * double differenceX = x - odometer.getX(); //get the X distance between current position and
     * the destination double differenceY = y - odometer.getY(); //get the Y distance between
     * current position and the destination
     * 
     * turnTo(Math.atan2(differenceY, differenceX)); //find the angle to reach the destination and
     * turn to that angle
     * 
     * double distance = Math.sqrt(Math.pow(differenceX, 2) + Math.pow(differenceY, 2)); // Distance
     * // to travel leftMotor.setSpeed(motorStraight); rightMotor.setSpeed(motorStraight +
     * weightCompensation); driveDist(distance);
     */
  }

  // ---------------------------------------METHODS---------------------------------------------

  // method to set the rotation speed of the robot
  public void setRotationSpeed(int speed) {
    int rotationSpeed = speed;
    int forwardSpeed = speed;
    setSpeeds(forwardSpeed, rotationSpeed);
  }

  // method to set the speed of the wheels
  public void setSpeeds(double forwardSpeed, double rotationSpeed) {
    double rightSpeed;
    double leftSpeed;

    leftSpeed = -rotationSpeed;
    rightSpeed = rotationSpeed;

    leftMotor.setSpeed((int) leftSpeed);
    rightMotor.setSpeed((int) rightSpeed);


    if (leftSpeed > 0) {
      leftMotor.forward();
    } else {
      leftMotor.backward();
    }

    if (rightSpeed > 0) {
      rightMotor.forward();
    } else {
      rightMotor.backward();
    }


  }

  // method to rotate the robot to a specific angle by making it turn until the correct angle
  public void rotateTo(double angle) {
    double[] pos = new double[3];
    boolean checkAngle = false;
    while (!checkAngle) {
      setRotationSpeed(100);
      odometer.getPosition(pos, new boolean[] {true, true, true}, 0, 0);
      double currentAngle = Math.toDegrees(odometer.getTheta());
      if (Math.abs(currentAngle - angle) < 1) {
        setRotationSpeed(0);
        checkAngle = true;
      }
    }
  }

  // method to turn the robot to an orientation with a minimal angle (different from above)
  public void turnTo(double theta) {
    double currentTheta = odometer.getTheta();
    double dTheta = theta - currentTheta;

    leftMotor.setSpeed(motorStraight);
    rightMotor.setSpeed(motorStraight);

    // rotating with minimal angle piece wise function
    if (dTheta < Math.PI * -1) {
      dTheta += 2 * Math.PI;
    }
    if (dTheta > Math.PI * 1) {
      dTheta -= 2 * Math.PI;
    }

    // clockwise turn
    if (dTheta < 0) {
      leftMotor.rotate(convertAngle(leftRadius, trackValue, Math.abs(dTheta) * 180 / Math.PI),
          true); // + works with localize
      rightMotor.rotate(-convertAngle(rightRadius, trackValue, Math.abs(dTheta) * 180 / Math.PI),
          false); // - works with localize
    }
    // counter clockwise turn
    if (dTheta > 0) {
      leftMotor.rotate(-convertAngle(leftRadius, trackValue, Math.abs(dTheta) * 180 / Math.PI),
          true); // -
      rightMotor.rotate(convertAngle(rightRadius, trackValue, Math.abs(dTheta) * 180 / Math.PI),
          false); // +
    }
  }

  // method to relocate to the waypoint
  public void relocateTo(double x, double y) {
    double differenceX = x - odometer.getX();
    double differenceY = y - odometer.getY();
    turnTo(Math.atan2(differenceY, differenceX));
    leftMotor.forward();
    rightMotor.forward();
  }


  // method to drive a specific distance
  public void driveDist(double distance) {
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), false);
  }

  // method returning the state of the robot whether it is navigating or not
  public boolean isItNavigating() {
    return this.isNavigating;
  }

  // method to set path
  public void setPath(int... intPath) {
    this.path = new double[4];

    for (int i = 0; i < intPath.length; i++) {
      path[i] = (intPath[i]) * Lab5.TILE_DIST /* + constantForRightQuadrant */;
    }
  }

  // method for converting angles
  public static int convertAngle(double radius, double width, double angle) {
    return (int) ((180 * Math.PI * width * angle / 360) / (Math.PI * radius));
  }

  // method for converting distances
  private static int convertDistance(double radius, double distance) {
    return (int) ((180 * distance) / (Math.PI * radius));
  }

  // method for staying in range of radians
  private double wrapAngle(double rads) {
    return ((rads % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
  }

  public void mountZip() {
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);

    armWheel.setSpeed(200);
    armWheel.forward();
    driveDist(160);
  }
}
