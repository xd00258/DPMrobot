package ca.mcgill.ecse211.project;

import lejos.robotics.SampleProvider;

/**
 * The UltrasonicPoller class implements a simple class
 * 
 * The UltrasonicPoller class implements a poller that will poll the robot's ultrasonic sensor and
 * send the data to be processed to a specified controller.
 *
 * @author Evan Laflamme, Karl Godin
 */

public class UltrasonicPoller {
  private SampleProvider us;
  private float[] usData;

  /**
   * UltrasonicPoller constructor.
   * 
   * @param us the sample provider to use
   * @param usData the array to store the data readings in
   */
  public UltrasonicPoller(SampleProvider us, float[] usData) {
    this.us = us;
    this.usData = usData;
  }

  /**
   * Fetches the samples from the sample provider, casts the sample to an integer and sends the
   * sample to be processed by the controller.
   * 
   * @return An int representing the distance read by the sensor in cm.
   */
  public int fetchSample() {
    us.fetchSample(usData, 0); // acquire data
    return (int) (usData[0] * 100.0); // extract from buffer, cast to int
  }
}
