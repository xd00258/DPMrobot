package ca.mcgill.ecse211.test.USFieldSwoop;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
  private SampleProvider us;
  private float[] usData;
  private boolean printFormattedData;
  private Odometer odometer;
  public static int data;
  
  public boolean done = false;
  

  public UltrasonicPoller(SampleProvider us, float[] usData, Odometer odometer, boolean printFormattedData) {
    this.us = us;
    this.usData = usData;
    this.odometer = odometer;
    this.printFormattedData = printFormattedData;
  }
  public void run() {
    while (!done) {
      us.fetchSample(usData, 0); // acquire data
      data = (int) (usData[0] * 100.0); // extract from buffer, cast to int

      if(printFormattedData) {
        System.out.println(Math.toDegrees(odometer.getTheta()) + "\t" + data);
      }

      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }
  }
  
  public int fetchSample() {
    us.fetchSample(usData, 0); // acquire data
    data = (int) (usData[0] * 100.0); // extract from buffer, cast to int
    return data;
  }
}
