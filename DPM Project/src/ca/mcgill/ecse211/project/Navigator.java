package ca.mcgill.ecse211.project;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * Navigator class is used to move the robot.
 * 
 * @author Karl Godin
 *
 */

public class Navigator {

  /** Left motor of the robot */
  private EV3LargeRegulatedMotor leftMotor;

  /** Right motor of the robot */
  private EV3LargeRegulatedMotor rightMotor;

  /** Arm motor of the robot for traversing the zipline */
  private EV3LargeRegulatedMotor armMotor;

  /** Odometer used by the travelTo method */
  private Odometer odometer;

  /**
   * Generates a navigator with motor accelerations at 500
   * 
   * @param leftMotor the leftMotor parameter represents the left motor
   * @param rightMotor the rightMotor parameter represents the right motor
   * @param armMotor the armMotor parameter represents the arm motor to traverse the zipline
   * @param odometer the odometer parameter is the odometer used by the navigator
   */
  public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      EV3LargeRegulatedMotor armMotor, Odometer odometer) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.armMotor = armMotor;

    this.odometer = odometer;

    // reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(1500);
    }
  }

  /**
   * Travels to the point (x,y)
   * 
   * @param x the x parameter represents the x position in cm the robot should go to
   * @param y the y parameter represents the y position in cm the robot should go to
   */
  public void travelTo(double x, double y) {
    double differenceX = (x * DPMProject.TILE_LENGTH) - odometer.getX(); // get the X distance
                                                                         // between current position
                                                                         // and the
    // destination
    double differenceY = (y * DPMProject.TILE_LENGTH) - odometer.getY(); // get the Y distance
                                                                         // between current position
                                                                         // and the
    // destination

    turnTo(Math.atan2(differenceY, differenceX)); // find the angle to reach the destination and
                                                  // turn to that angle

    double distance = Math.sqrt(Math.pow(differenceX, 2) + Math.pow(differenceY, 2)); // Distance
                                                                                      // to travel
    leftMotor.setSpeed(DPMProject.FORWARD_SPEED);
    rightMotor.setSpeed((int) (DPMProject.FORWARD_SPEED * DPMProject.MOTOR_BALANCE));
    driveDist(distance);
  }

  /**
   * Drives a set distance.
   * 
   * @param distance The distance to be travelled in cm.
   */
  public void driveDist(double distance) {
    leftMotor.rotate(convertDistance(DPMProject.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(DPMProject.WHEEL_RADIUS, distance), false);
  }

  /**
   * Turns to a given angle
   * 
   * @param theta The angle by which the robot must be turned.
   */
  public void turnTo(double theta) {
    double currentTheta = odometer.getTheta();
    double dTheta = theta - currentTheta;

    leftMotor.setSpeed(DPMProject.ROTATE_SPEED);
    rightMotor.setSpeed((int) (DPMProject.ROTATE_SPEED * DPMProject.MOTOR_BALANCE));

    // rotating with minimal angle piece wise function
    if (dTheta < Math.PI * -1) {
      dTheta += 2 * Math.PI;
    }
    if (dTheta > Math.PI * 1) {
      dTheta -= 2 * Math.PI;
    }

    // clockwise turn
    if (dTheta < 0) {
      dTheta += 0.0056 * dTheta; // 0.0056

      // odometer.setTheta(odometer.getTheta()/* + Math.toRadians(0.75)*/);
      // dTheta += Math.toRadians(0.75);

      leftMotor.rotate(convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(dTheta)),
          true); // + works with localize
      rightMotor.rotate(-convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(dTheta)),
          false); // - works with localize
    }
    // counter clockwise turn
    if (dTheta > 0) {
      dTheta -= 0.0056 * dTheta;

      // odometer.setTheta(odometer.getTheta() /*- Math.toRadians(1)*/);
      // dTheta -= Math.toRadians(1);

      leftMotor.rotate(-convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(dTheta)),
          true); // -
      rightMotor.rotate(convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(dTheta)),
          false); // +
    }
  }

  /**
   * Travels forward with the arm motor activated in order to mount and cross the zip line.
   * 
   * @param diagonal States whether or not the zip line is diagonal
   */
  public void mountAndTraverseZipline(boolean diagonal) {
    boolean offGround = false;

    setMotorSpeeds(200);
    leftMotor.forward();
    rightMotor.forward();

    armMotor.setSpeed(350);
    armMotor.setAcceleration(200);
    armMotor.forward();
    EV3ColorSensor colorSensor = DPMProject.lightLocalizer.detector.getColorSensor();

    colorSensor.setCurrentMode("RGB");
    float[] data = new float[3];
    colorSensor.fetchSample(data, 0);

    float ave = (data[0] + data[1] + data[2]) / 3;

    // Determines if the robot is off the ground
    while (true) {
      colorSensor.fetchSample(data, 0);

      // Computes average of RGB
      ave = (data[0] + data[1] + data[2]) / 3;

      // If the average is above 0.015, the robot is off the ground and must 4 seconds before
      // starting to poll again
      if (ave < 0.015) {
        offGround = true;

        try {
          Thread.sleep(4000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }

      // If the robot was off the ground and is now close to the ground, it starts turning to the
      // right to avoid the zip line
      if (offGround && ave > 0.015) {
        leftMotor.setSpeed(230);
        rightMotor.setSpeed(215);
        leftMotor.forward();
        rightMotor.forward();

        break;
      }

      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    colorSensor.setCurrentMode("Red");

    // Travels for longer if the zip line is diagonal
    if (diagonal) {
      try {
        Thread.sleep(2100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      leftMotor.setSpeed(215);
      rightMotor.setSpeed(245);
      leftMotor.forward();
      rightMotor.forward();
    } else {
      try {
        Thread.sleep(4000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    armMotor.stop();
  }

  /**
   * Rotates robot clockwise or anti clockwise
   * 
   * @param clockwise the clockwise parameter is a boolean that determines if the robot is going
   *        left or right
   */
  public void rotate(boolean clockwise) {
    leftMotor.setSpeed(DPMProject.ROTATE_SPEED);
    rightMotor.setSpeed((int) (DPMProject.ROTATE_SPEED * DPMProject.MOTOR_BALANCE));
    if (!clockwise) {
      leftMotor.backward();
      rightMotor.forward();
    } else {
      leftMotor.forward();
      rightMotor.backward();
    }
  }

  /**
   * Rotates the robot at a given speed for a certain amount
   * 
   * @param clockwise the direction in which the robot must rotate
   * @param radians the amount it must rotate
   * @param speed the speed at which it must rotate
   */
  public void rotate(boolean clockwise, double radians, int speed) {
    // Set the speed of the motor with offset
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed((int) (speed * DPMProject.MOTOR_BALANCE));

    // Rotate the motors a converted amount
    if (!clockwise) {
      leftMotor.rotate(-convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(radians)),
          true);
      rightMotor.rotate(convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(radians)),
          true);
    } else {
      leftMotor.rotate(convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(radians)),
          true);
      rightMotor.rotate(-convertAngle(DPMProject.WHEEL_RADIUS, DPMProject.TRACK, Math.abs(radians)),
          true);
    }
  }

  /**
   * Rotates the robot indefinitely at a given speed
   * 
   * @param clockwise The direction in which the robot must turn
   * @param speed The speed at which it must turn
   */
  public void rotate(boolean clockwise, int speed) {
    // Set the speed of the motors with offset
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed((int) (speed * DPMProject.MOTOR_BALANCE));
    if (!clockwise) {
      leftMotor.backward();
      rightMotor.forward();
    } else {
      leftMotor.forward();
      rightMotor.backward();
    }
  }

  /**
   * Sets the speeds of the motors
   * 
   * @param speed The speed of the motors
   */
  public void setMotorSpeeds(int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed((int) (speed * DPMProject.MOTOR_BALANCE));
  }

  /**
   * Ensures the given angle is always between -180 to 180 so as to be the smallest
   * 
   * @param angle the angle parameter is the angle in degrees to be corrected
   * @return the corrected angle in degrees
   */
  public static double correctAngleChange(double angle) {
    if (angle > 180) {
      return angle - 360;
    } else if (angle < -180) {
      return angle + 360;
    } else {
      return angle;
    }
  }

  /**
   * Converts robot displacement into wheel distance
   * 
   * @param radius the radius parameter represents the radius of the wheel
   * @param distance the distance parameter is the distance that should be moved by the robot
   * @return
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * Converts the robot angle change into wheel distance
   * 
   * @param radius the radius parameter is the radius of the wheel
   * @param width the width parameter is the distance between both wheels
   * @param angle the angle parameter is the angle change in degrees
   * @return
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, width * angle / 2);
  }

  /**
   * Both wheels are set to forward at forwardSpeed
   * 
   * @param forwardSpeed the forwardSpeed parameter is the speed of the wheels while going forward
   */
  public void moveForward(int forwardSpeed) {
    leftMotor.setSpeed(forwardSpeed);
    rightMotor.setSpeed(forwardSpeed);
    leftMotor.forward();
    rightMotor.forward();
  }

  /**
   * Both wheels are set to forward with preset speed
   */
  public void moveForward() {
    leftMotor.setSpeed(DPMProject.FORWARD_SPEED);
    rightMotor.setSpeed(DPMProject.FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
  }


  /**
   * Both wheels are set to backward at backwardSpeed
   * 
   * @param backwardSpeed the backwardSpeed parameter is the speed at which the robot goes backward
   */
  public void moveBackward(int backwardSpeed) {
    leftMotor.setSpeed(backwardSpeed);
    rightMotor.setSpeed(backwardSpeed);
    leftMotor.backward();
    rightMotor.backward();
  }

  /**
   * Both wheels are set to backward with preset speed
   */
  public void moveBackward() {
    leftMotor.setSpeed(DPMProject.BACKWARD_SPEED);
    rightMotor.setSpeed(DPMProject.BACKWARD_SPEED);
    leftMotor.backward();
    rightMotor.backward();
  }

  /**
   * Stops both motors simultaneously
   */
  public void stopMotors() {
    leftMotor.stop(true);
    rightMotor.stop();
  }


  /**
   * Returns leftMotor object
   * 
   * @return leftMotor object
   */
  public EV3LargeRegulatedMotor getLeftMotor() {
    return leftMotor;
  }

  /**
   * Sets the leftMotor object
   * 
   * @param leftMotor the leftMotor parameter is the left motor to be set
   */
  public void setLeftMotor(EV3LargeRegulatedMotor leftMotor) {
    this.leftMotor = leftMotor;
  }

  /**
   * Returns the rightMotor object
   * 
   * @return the rightMotor object
   */
  public EV3LargeRegulatedMotor getRightMotor() {
    return rightMotor;
  }

  /**
   * Sets the rightMotor object
   * 
   * @param rightMotor the leftMotor parameter is the left motor to be set
   */
  public void setRightMotor(EV3LargeRegulatedMotor rightMotor) {
    this.rightMotor = rightMotor;
  }
}
