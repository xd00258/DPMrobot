package ca.mcgill.ecse211.project;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The Odometer class implements an odometer to keep track of the position and heading of the robot.
 * The odometer runs in a seperate thread to ensure constant odometry updates. The getter and setter
 * methods within the odometer class use a lock object to ensure thread-safety when being accessed
 * by other threads.
 *
 * @author Evan Laflamme
 */

public class Odometer extends Thread {
  /** The x position of the robot */
  private double x;
  /** The y position of the robot */
  private double y;
  /** The heading of the robot */
  private double theta;

  /** The left motor of the robot */
  private EV3LargeRegulatedMotor leftMotor;
  /** The right motor of the robot */
  private EV3LargeRegulatedMotor rightMotor;

  /** The current left motor tacho count */
  private double leftMotorTachoCount;
  /** The current right motor tacho count */
  private double rightMotorTachoCount;

  /** The odometer update period, in ms */
  private static final long ODOMETER_PERIOD = 25;

  /** Lock object for mutual exclusion */
  private Object lock;

  /**
   * Odometer constructor.
   * 
   * @param leftMotor a reference to the left motor of the robot
   * @param rightMotor a reference to the right motor of the robot
   */
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    // Initialize references to the robot's motors
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Initialize the position of the robot
    this.x = 0.0;
    this.y = 0.0;
    this.theta = Math.PI / 2;

    // Initialize the lock object
    lock = new Object();
  }

  /**
   * The run method performs the actual odometry calculations for the robot. Every correction
   * period, the odometer will get the tacho counts from the left and right motors in order to
   * calculate the new position and heading of the robot.
   */
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Get current tacho counts
      double nowleftMotorTachoCount = Math.toRadians(leftMotor.getTachoCount());
      double nowRightMotorTachoCount =
          Math.toRadians(rightMotor.getTachoCount() / DPMProject.MOTOR_BALANCE);

      // Compute the tacho count differences for each motor
      double dLeftTacho = nowleftMotorTachoCount - getLeftMotorTachoCount();
      double dRightTacho = nowRightMotorTachoCount - getRightMotorTachoCount();

      // Compute the wheel displacements
      double dLeftArc = dLeftTacho * DPMProject.WHEEL_RADIUS;
      double dRightArc = dRightTacho * DPMProject.WHEEL_RADIUS;

      // Compute the displacement of the robot
      double deltaD = (dLeftArc + dRightArc) / 2;

      // Compute the change in heading of the robot
      double deltaTheta = (dLeftArc - dRightArc) / DPMProject.TRACK;

      // Compute the change in x and y of the robot
      double deltaX = deltaD * Math.cos(getTheta() + deltaTheta);
      double deltaY = deltaD * Math.sin(getTheta() + deltaTheta);

      // Update latest tachometer readings
      setLeftMotorTachoCount(nowleftMotorTachoCount);
      setRightMotorTachoCount(nowRightMotorTachoCount);

      synchronized (lock) {
        // Update new position and
        setX(getX() + deltaX);
        setY(getY() + deltaY);
        setTheta(getTheta() - deltaTheta);

        // Ensure that theta is within 0 to 360 degrees
        if (getTheta() >= 2 * Math.PI) {
          setTheta(getTheta() - 2 * Math.PI);
        } else if (theta < 0) {
          setTheta(getTheta() + 2 * Math.PI);
        }
      }

      // Ensure that the odometer only runs once per period
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

  /**
   * This method gets the current position of the robot. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @param position an array to store the position in
   * @param update an array indicating which position entries to update
   */
  public void getPosition(double[] position, boolean[] update) {
    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = Math.toDegrees(theta);
    }
  }

  /**
   * This method gets the current value of x. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @return x
   */
  public double getX() {
    double result;

    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      result = x;
    }

    return result;
  }

  /**
   * This method gets the current value of y. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @return y
   */
  public double getY() {
    double result;

    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      result = y;
    }

    return result;
  }

  /**
   * This method gets the current value of theta. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @return theta
   */
  public double getTheta() {
    double result;

    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  /**
   * This method gets the current value of theta in degrees. This method uses the lock object to
   * ensure thread-safety.
   * 
   * @return theta in degrees
   */
  public double getThetaDegree() {
    double result;

    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      result = Math.toDegrees(theta);
    }

    return result;
  }

  /**
   * This method sets the current position of the robot. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @param position an array containing the new position values
   * @param update an array indicating which position entries to update
   */
  public void setPosition(double[] position, boolean[] update) {
    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  /**
   * This method sets the current value of x. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @param x the new value of x
   */
  public void setX(double x) {
    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      this.x = x;
    }
  }

  /**
   * This method sets the current value of y. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @param y the new value of y
   */
  public void setY(double y) {
    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      this.y = y;
    }
  }

  /**
   * This method sets the current value of theta. This method uses the lock object to ensure
   * thread-safety.
   * 
   * @param x the new value of theta
   */
  public void setTheta(double theta) {
    // Ensure that the values don't change while the odometer is running
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * This method gets the current tacho count for the left motor of the robot.
   * 
   * @return the leftMotorTachoCount
   */
  public double getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * This method sets the current tacho count for the left motor of the robot. This method uses the
   * lock object to ensure thread-safety.
   * 
   * @param leftMotorTachoCount the new value of leftMotorTachoCount
   */
  public void setLeftMotorTachoCount(double leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * This method gets the current tacho count for the right motor of the robot.
   * 
   * @return the rightMotorTachoCount
   */
  public double getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * This method sets the current tacho count for the right motor of the robot. This method uses the
   * lock object to ensure thread-safety.
   * 
   * @param rightMotorTachoCount the new value of rightMotorTachoCount
   */
  public void setRightMotorTachoCount(double rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
