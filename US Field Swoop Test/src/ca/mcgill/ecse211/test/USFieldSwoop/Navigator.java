package ca.mcgill.ecse211.test.USFieldSwoop;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator {
  private EV3LargeRegulatedMotor leftMotor;

  private EV3LargeRegulatedMotor rightMotor;

 private final double WHEEL_RADIUS = USFieldSwoop.WHEEL_RADIS; // Specifications of the robot car
  private final double TRACK_VALUE = USFieldSwoop.TRACK;
  private int rotateSpeed = 200;

  public Navigator(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, int rotateSpeed) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.rotateSpeed = rotateSpeed;
  }

  /** Amount of degrees to rotate the robot */
  public void rotate(double degrees) {
    leftMotor.setSpeed(rotateSpeed);
    rightMotor.setSpeed(rotateSpeed);

    // Determines the wheel rotation based on angle change
    int convertedAngle = convertAngle(WHEEL_RADIUS, TRACK_VALUE, degrees);

    // Rotates robot
    leftMotor.rotate(convertedAngle, true);
    rightMotor.rotate(-convertedAngle, false);
  }

  /** Converts robot displacement into wheel distance */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /** Converts the robot angle change into wheel distance */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /** Sets the rotation speed of the robot */
  public void setRotationSpeed(int speed) {
    rotateSpeed = speed;
  }

}
