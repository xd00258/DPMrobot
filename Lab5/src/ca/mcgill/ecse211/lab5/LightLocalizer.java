package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer extends Thread {

  public static int ROTATION_SPEED = 100;
  private Odometer odometer;
  public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);


  private boolean firstLocalize = false;

  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private double sensorPositionX = 14.5;
  private double sensorPositionY = 14.5;

  private double forwardSpeed = 0;
  private double rotationSpeed = 0;

  public LightLocalizer(Odometer odometer, UltrasonicPoller usPoller,
      EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigator nav) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    colorSensor.setFloodlight(true);

  }

  public void doLocalizationOnly() {

    double nearestIntersectX;
    double nearestIntersectY;
    double angleCorrectCorner = 0;

    if (!firstLocalize) {



      // double angle[] = new double[4];

      nearestIntersectX = 30.48 * PathGenerator.getStartX(Lab5.startingPosition);
      nearestIntersectY = 30.48 * PathGenerator.getStartY(Lab5.startingPosition);

      if (Lab5.startingPosition == 0) {
        angleCorrectCorner = 0;
        sensorPositionX = -sensorPositionX;
        sensorPositionY = -sensorPositionY;
        // stupidCorrection = Math.PI;
      }
      if (Lab5.startingPosition == 1) {
        angleCorrectCorner = 90;
        sensorPositionY = -sensorPositionY;
        // stupidCorrection = Math.PI/2;
      } else if (Lab5.startingPosition == 2) {
        angleCorrectCorner = 180;
        // stupidCorrection = Math.PI / 2;
      } else if (Lab5.startingPosition == 3) {
        angleCorrectCorner = 270;
        sensorPositionX = -sensorPositionX;
        // stupidCorrection = Math.PI/2;
      }
    }

    else {
      double cX = Math.round(odometer.getX() / 30);
      double cY = Math.round(odometer.getY() / 30);
      sensorPositionX = -14.5;
      sensorPositionY = -14.5;
      // double angle[] = new double[4];
      angleCorrectCorner = 0;

      nearestIntersectX = 30.48 * cX;
      nearestIntersectY = 30.48 * cY;
    }



    int counter = 0; // counter variable to keep track of how many lines detected
    int index = 0; // index to access angle array
    double[] angle = new double[4]; // array to store the 4 angles saved from light localizing
    setSpeeds(50, -50); // rotation speed of wheels

    // getting the 4 angles (each at the respective line detection)
    while (counter < 4) {
      if (colorSensor.getColorID() >= 13) {
        double[] pos = new double[3];
        odometer.getPosition(pos, new boolean[] {true, true, true}, 0, 0);
        angle[index] = pos[2];
        Sound.beep();
        try {
          Thread.sleep(500);
        } catch (InterruptedException ex) {
          // none
        }
        index++;
        counter++;
      }
    }

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

    double omega = angle[0] - angle[1];
    double alpha = angle[1] - angle[2];
    double beta = angle[2] - angle[3];

    double dy = 0;
    double dx = 0;

    if (!firstLocalize) {
      if (beta < alpha) {
        dy = -sensorPositionY * Math.cos(thetaX / 2); // +nearestIntersect
      } else {
        dy = sensorPositionY * Math.cos(thetaX / 2); // +nearestIntersect
      }

      if (omega < alpha) {
        dx = -sensorPositionX * Math.cos(thetaY / 2); // +nearestIntersect
      } else {
        dx = sensorPositionX * Math.cos(thetaY / 2); // +nearestIntersect
      }
    } else {
      if (beta > alpha) {
        dy = -sensorPositionY * Math.cos(thetaX / 2); // +nearestIntersect
      } else {
        dy = sensorPositionY * Math.cos(thetaX / 2); // +nearestIntersect
      }

      dy = 0;
      if (omega < alpha) {
        dx = -sensorPositionX * Math.cos(thetaY / 2); // +nearestIntersect
      } else {
        dx = sensorPositionX * Math.cos(thetaY / 2); // +nearestIntersect
      }
    }

    // computing the real x and y position of the robot relative to the 0, 0
    double x = dx + nearestIntersectX; // +nearestIntersect
    double y = dy + nearestIntersectY; // +nearestIntersect


    double dTheta = 180 - (angle[0] + angle[2]) / 2; // added

    double theta = Math.toDegrees(odometer.getTheta()) + dTheta; // added


    if (!firstLocalize) {
      odometer.setPosition(
          new double[] {x, y,
              (theta) * Math.PI / 180 - Math.PI / 2 + Math.toRadians(angleCorrectCorner)},
          new boolean[] {true, true, true});
    } else {
      odometer.setPosition(new double[] {x, y, odometer
          .getTheta()/* (theta) * Math.PI / 180 - Math.PI/2 */ /*- Math.PI/2 - stupidCorrection*/},
          new boolean[] { // was originally only - pi/2
              true, true, true});
    }

    firstLocalize = true;

    try {
      Thread.sleep(1500);
    } catch (InterruptedException ex) {
      // none
    }
  }

  // ---------------------------------------- METHODS
  // --------------------------------------------------

  // method to set speed of the wheels
  public void setSpeeds1(double forwardSpeed, double rotationSpeed) {
    double rightSpeed;
    double leftSpeed;
    this.forwardSpeed = forwardSpeed;
    this.rotationSpeed = rotationSpeed;

    leftSpeed = forwardSpeed;
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

  // method to rotate the robot to a specific angle by making it turn until the correct theta is
  // compared
  public void rotateTo(double angle) {
    double[] pos = new double[3];
    boolean checkAngle = false;
    while (!checkAngle) {
      setRotationSpeed(-ROTATION_SPEED);
      odometer.getPosition(pos, new boolean[] {true, true, true}, 0, 0);
      double currentAngle = Math.toDegrees(odometer.getTheta());
      if (Math.abs(currentAngle - angle) < 1) {
        setRotationSpeed(0);
        checkAngle = true;
      }
    }
  }

  // set the rotation speed of the wheels
  public void setRotationSpeed(int speed) {
    rotationSpeed = speed;
    setSpeeds(forwardSpeed, rotationSpeed);
  }

  // method to set the speed of the two motors
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

  // method to make the robot drive forward for a certain amount of time
  public void driveForward() {
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);
    leftMotor.forward();
    rightMotor.forward();
    try {
      Thread.sleep(2700);
    } catch (InterruptedException ex) {
      // none//
    }
    leftMotor.stop();
    rightMotor.stop();
  }
}
