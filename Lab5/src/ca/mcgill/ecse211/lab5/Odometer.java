package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;

public class Odometer extends Thread {
  // robot position
  private double x;
  private double y;
  private double theta;
  private double leftMotorTachoCount;
  private double rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private double thetaDegree;
  private double yFlipped = 0.0;

  private final double wheelRadius = 2.1;
  private final double track = Lab5.TRACK;

  private static final long ODOMETER_PERIOD = 25; /* odometer update period, in ms */

  private Object lock; /* lock object for mutual exclusion */

  // default constructor
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = Math.PI / 2;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();
  }

  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // get current tacho
      double leftTacho = Math.toRadians(leftMotor.getTachoCount());
      double rightTacho = Math.toRadians(rightMotor.getTachoCount());

      // find difference tacho
      double dLeftTacho = leftTacho - leftMotorTachoCount;
      double dRightTacho = rightTacho - rightMotorTachoCount;

      // computing wheel displacements
      double dLeftArc = dLeftTacho * wheelRadius;
      double dRightArc = dRightTacho * wheelRadius;

      // computing vehicle displacement
      double deltaD = (dLeftArc + dRightArc) / 2;

      // computing the angle made
      double deltaTheta = (dLeftArc - dRightArc) / track;

      // computing change in x and y
      double deltaX = deltaD * Math.cos(theta + deltaTheta);
      double deltaY = deltaD * Math.sin(theta + deltaTheta);

      // update last tachometer
      leftMotorTachoCount = leftTacho;
      rightMotorTachoCount = rightTacho;

      synchronized (lock) {
        /**
         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
         * and theta in this block. Do not perform complex math
         * 
         */
        x = x + deltaX;
        // yFlipped = yFlipped + deltaY;
        y = y + deltaY;
        theta = theta - deltaTheta;


        if (theta >= 2 * Math.PI) {
          theta = theta - 2 * Math.PI;
        } else if (theta < 0) {
          theta = theta + 2 * Math.PI;
        }

        thetaDegree = (theta * 180 / Math.PI); // converting radians into degrees

        // keep the range of degrees [0, 360)
        // if (thetaDegree >= 360)
        // thetaDegree = thetaDegree - 360;
        // else if (thetaDegree < 0)
        // thetaDegree = thetaDegree + 360;
      }

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }
    }
  }

  public void getPosition(double[] position, boolean[] update, double correctionX,
      double correctionY) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x; // correcting relative to the 0, 0 of the board
      if (update[1])
        position[1] = y; // correcting relative to the 0, 0 of the board
      if (update[2])
        position[2] = thetaDegree;
    }
  }

  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  // mutators
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * @return the leftMotorTachoCount
   */
  public double getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @param leftMotorTachoCount the leftMotorTachoCount to set
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * @return the rightMotorTachoCount
   */
  public double getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * @param rightMotorTachoCount the rightMotorTachoCount to set
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
