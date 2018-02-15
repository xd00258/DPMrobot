package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class USLocalizer {

  public enum LocalizerType {
    FALLING_EDGE, RISING_EDGE
  };

  public static int ROTATION_SPEED = 100;
  private Odometer odometer;
  private UltrasonicPoller usPoller;
  private LocalizerType locType;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private Navigator nav;

  private double forwardSpeed = 0;
  private double rotationSpeed = 0;

  private double leftRadius, rightRadius = 2.1;
  private double track = 15; // 16

  private boolean leftWallDetect = false;
  private boolean rightWallDetect = false;
  private boolean facingWall = false;

  // rising and falling edge conditions before detection
  private double risingEdge = 45;
  private double fallingEdge = 30;
  private double detectWall = 50;

  // constructor
  public USLocalizer(Odometer odometer, UltrasonicPoller usPoller, LocalizerType type,
      EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigator nav) {
    this.odometer = odometer;
    this.usPoller = usPoller;
    this.locType = type;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.nav = nav;

    usPoller.start();
  }

  // Main function of USLocalizer
  public void doLocalization() {
    // usPoller.start();
    double[] pos = new double[3]; // array to contain the position of the robot, x, y and theta
    double angleA = 0; // first angle found when turning to left wall
    double angleB = 0; // second angle found when turning to right wall

    // FALLING EDGE
    if (locType == LocalizerType.FALLING_EDGE) {
      // if the robot starts facing a wall, turn until it doesnt face a wall
      if (UltrasonicPoller.data <= detectWall) {
        facingWall = true;
        setRotationSpeed(ROTATION_SPEED);

        while (facingWall) {
          setRotationSpeed(ROTATION_SPEED);
          if (UltrasonicPoller.data > fallingEdge) {
            facingWall = false;
          }
        }
        try {
          Thread.sleep(2000);
        } catch (InterruptedException ex) {
          // none
        }

      }

      // falling edge localization starts here
      while (!leftWallDetect) {
        // turn until wall detected and save the orientation of the robot to angleA
        setRotationSpeed(ROTATION_SPEED);

        if (UltrasonicPoller.data < fallingEdge) {
          leftWallDetect = true;
          angleA = Math.toDegrees(odometer.getTheta());
          Sound.beep();
        }
      }

      setRotationSpeed(-ROTATION_SPEED);
      try {
        Thread.sleep(2000);
      } catch (InterruptedException ex) {
        // none
      }
      // turn the other way until second wall is detected and save the orientation of the robot to
      // angle B
      while (!rightWallDetect) {
        setRotationSpeed(-ROTATION_SPEED);

        if (UltrasonicPoller.data < fallingEdge) {
          rightWallDetect = true;
          angleB = Math.toDegrees(odometer.getTheta());
          Sound.beep();
        }
      }
      // stop and compute orientation
      setRotationSpeed(0);
      odometer.getPosition(pos, new boolean[] {true, true, true}, 0, 0);
      double dTheta;

      // computing correct heading, must find constants EXPERIMENTALLY
      if (angleA > angleB) {
        dTheta = 45 - (angleA + angleB) / 2;
      } else {
        dTheta = 225 - (angleA + angleB) / 2;
      }
      double adjustedTheta = dTheta + pos[2];

      // update the odometer's position and angle and correcting it experimentally
      odometer.setPosition(new double[] {0, 0, Math.toRadians(adjustedTheta)},
          new boolean[] {true, true, true});
      rotateTo(90);

      odometer.setPosition(new double[] {0, 0, Math.PI / 2}, new boolean[] {true, true, true});
      try {
        Thread.sleep(2000);
      } catch (InterruptedException ex) {
        // none
      }

      rotateTo(45);
      driveForward();

    }

    // RISING EDGE
    else {
      // if the robot starts and it is not facing a wall, turn until its facing a wall
      if (UltrasonicPoller.data >= detectWall) {
        facingWall = false;
        setRotationSpeed(ROTATION_SPEED);
        while (!facingWall) {
          setRotationSpeed(ROTATION_SPEED);
          if (UltrasonicPoller.data < fallingEdge) {
            facingWall = true;
          }
        }
        try {
          Thread.sleep(2000);
        } catch (InterruptedException ex) {
          // none
        }
      }

      // rising edge localization starts here
      while (!leftWallDetect) {
        // turn until first wall is detected and save the orientation to angleA
        setRotationSpeed(ROTATION_SPEED);

        if (UltrasonicPoller.data > risingEdge) {
          odometer.getPosition(pos, new boolean[] {true, true, true}, 0, 0);
          leftWallDetect = true;
          angleA = Math.toDegrees(odometer.getTheta());
          Sound.beep();
        }
      }
      setRotationSpeed(-ROTATION_SPEED);
      try {
        Thread.sleep(2000);
      } catch (InterruptedException ex) {
        // none
      }

      // turn the other way until the second wall is detected and save the orientation to angleB
      while (!rightWallDetect) {
        setRotationSpeed(-ROTATION_SPEED);

        if (UltrasonicPoller.data > risingEdge) {
          rightWallDetect = true;
          angleB = Math.toDegrees(odometer.getTheta());
          Sound.beep();
        }
      }
      // stop and compute orientations
      setRotationSpeed(0);
      odometer.getPosition(pos, new boolean[] {true, true, true}, 0, 0);
      double dTheta;

      // computing correct heading with constants found EXPERIMENTALLY
      if (angleA >= angleB) {
        dTheta = 45 - (angleA + angleB) / 2;
      } else {
        dTheta = 225 - (angleA + angleB) / 2;
      }
      double adjustedTheta = dTheta + pos[2];

      wrapAngle(Math.toRadians(adjustedTheta));

      // update odometer position and correct it
      odometer.setPosition(new double[] {0, 0, Math.toRadians(adjustedTheta)},
          new boolean[] {true, true, true});
      rotateTo(0);
      try {
        Thread.sleep(2000);
      } catch (InterruptedException ex) {
        // none
      }

    }
  }


  // ----------------------METHODS ---------------------

  // method to drive forward for a specific time
  public void driveForward() {
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);
    leftMotor.forward();
    rightMotor.forward();
    try {
      Thread.sleep(2000);
    } catch (InterruptedException ex) {
      // none//
    }
    leftMotor.stop(true);
    rightMotor.stop();
  }

  // method to set the rotation speed of the robot
  public void setRotationSpeed(int speed) {
    rotationSpeed = speed;
    setSpeeds(forwardSpeed, rotationSpeed);
  }

  // method to set the speed to the wheels of the robot
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

  // method to rotate the robot to a specific angle by turning until the correct angle is found
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

  // method to prevent going off the 2 pi range
  private double wrapAngle(double rads) {
    return ((rads % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
  }

  // method to convert distance with wheel rotation
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  // method to convert angle with distance
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  // method to turn to an angle with minimal angle rotation (different from above's method)
  public void turnTo(double theta) {
    double currentTheta = odometer.getTheta();
    double dTheta = theta - currentTheta;
    leftMotor.setSpeed(ROTATION_SPEED);
    rightMotor.setSpeed(ROTATION_SPEED);

    // rotating with minimal angle piece wise function
    if (dTheta < Math.PI * -1) {
      dTheta += 2 * Math.PI;
    }
    if (dTheta > Math.PI * 1) {
      dTheta -= 2 * Math.PI;
    }

    // clockwise turn
    if (dTheta < 0) {
      leftMotor.rotate(convertAngle(leftRadius, track, Math.abs(dTheta) * 180 / Math.PI), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, Math.abs(dTheta) * 180 / Math.PI), false);
    }
    // counter clockwise turn
    if (dTheta > 0) {
      leftMotor.rotate(-convertAngle(leftRadius, track, Math.abs(dTheta) * 180 / Math.PI), true);
      rightMotor.rotate(convertAngle(rightRadius, track, Math.abs(dTheta) * 180 / Math.PI), false);
    }
  }
}
