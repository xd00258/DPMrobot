package ca.mcgill.ecse211.project;


import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * The WallDetector class polls the ultrasonic sensor and applies a filter to it.
 * 
 * @author Karl Godin
 *
 */

public class WallDetector {
  public static UltrasonicPoller usPoller;

  int filterControl;
  private final int FILTER_OUT = 300;
  int lastValidSample;

  /**
   * WallDetector Constructor
   * 
   * @param usport The port of the EV3UltrasonicSensor.
   */
  public WallDetector(Port usport) {
    SensorModes usSensor = null;
    boolean notSet = true;

    while (notSet) { // Ensure that we never get a port error
      try {
        usSensor = new EV3UltrasonicSensor(usport);
        notSet = false;
      } catch (Exception e) {
        // Do nothing
      }
    }
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    usPoller = new UltrasonicPoller(usDistance, usData);
  }

  /**
   * Gets a sample distance
   * 
   * @return the distance the sensor has sensed
   */
  public int getSample() {
    try {
      Thread.sleep(50);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    int sample = usPoller.fetchSample();

    int filteredSample = 0;

    // Reused filtering code from previous labs
    if (sample >= 150 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
      return lastValidSample;
    } else if (sample >= 150) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      filteredSample = sample;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      filteredSample = sample;
    }

    lastValidSample = filteredSample;

    return filteredSample;
  }

  /** Calibrates the sensor */
  public void calibrateSensor() {
    // Reads a few values so that if there is nothing in front of it, the filter does not
    // automatically discard it as 0
    for (int i = 0; i < 30; i++) {
      getSample();
    }
  }


}
