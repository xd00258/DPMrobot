package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * The LineDetector class is a utility class which encapsulates everything related to line
 * detection. A few methods can be used to perform basic tasks related to line detection.
 * 
 * @author Karl Godin
 *
 */

public class LineDetector {
  /** Time interval between each line detection in ms */
  private static final long CORRECTION_PERIOD = 10;

  /** Color sensor used to detect light intensity */
  private SampleProvider colorSensor;

  private EV3ColorSensor sensor;
  /** Used to store the color intensity detected by the light sensor */
  private float[] colorSample;

  /** Array of color samples used to generate an average */
  private float[] recentValues;
  /** Size of recentValues */
  private int valueSize = 25;
  /** Previous average used in hasLine method */
  private float prevAve = 0;
  /** The position in the array. Used to implement a looping array. */
  private int filterPos = 0;
  /** The size of data points to take into account when performing the moving average. */
  private int filterWidth = 5;

  /**
   * The relative difference from the previous average the current average has to be to detect a
   * line
   */
  private float threshold = 0.25f;

  /** Used to count the current count related to the delay */
  private int calibrationCounter = 0;
  /** The number of counts that must be performed until the sensor is calibrated */
  private int calibDelay = 20;

  /** Used to count the current count related to the stability of the readings */
  private int stableCounter = 0;
  /** The number of counts that must be performed until the sensor's readings are stabilized */
  private int stableDelay = 50;

  /** The time at the beginning of the hasLine method */
  private long correctionStart;
  /** The time at the end of the hasLine method */
  private long correctionEnd;

  /**
   * Constructor for the LineDetector class.
   * 
   * @param sensorPort Port in which the light sensor is plugged into. e.g.: S2.
   */
  @SuppressWarnings("resource")
  public LineDetector(String sensorPort) {
    sensor = new EV3ColorSensor(LocalEV3.get().getPort(sensorPort));
    colorSensor = sensor.getRedMode();

    colorSample = new float[colorSensor.sampleSize()];

    recentValues = new float[valueSize];

    // calibrateSensor();
  }

  /**
   * The hasLine method first fetches a color sample. It then computes a moving average of the
   * current value. If the calibration and stable counters are high enough, it performs light
   * detection. When performing light detection, it compares the current value to the previous. If
   * the difference is large enough (e.g.: 20% larger), a line is detected. The method then sleeps
   * the current thread for no more than the CORRECTION_PERIOD.
   * 
   * @return true if a line has been detected. false if a line has not been detected.
   */
  public boolean hasLine() {
    boolean hasLine = false;

    correctionStart = System.currentTimeMillis();

    colorSensor.fetchSample(colorSample, 0);

    float curSample = colorSample[0] * 100;

    float ave =
        prevAve + (curSample - recentValues[getFilterPos(-filterWidth)]) / ((float) filterWidth);

    if (calibrationCounter > calibDelay) {
      if (Math.abs(recentValues[filterPos] - ave) > (threshold * ave)
          && stableCounter > stableDelay) {
        // LocalEV3.get().getAudio().systemSound(0);
        hasLine = true;
        stableCounter = 0;
      } else {
        stableCounter++;
        if (stableCounter > stableDelay * 4) {
          stableCounter = stableDelay * 4;
        }
      }
    } else {
      calibrationCounter++;
    }


    recentValues[filterPos] = ave;

    prevAve = ave;

    // Increments filterPos by 1 and loops it at valueSize
    filterPos = getFilterPos(1);



    // this ensure the odometry correction occurs only once every period
    correctionEnd = System.currentTimeMillis();
    if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
      try {
        Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      } catch (InterruptedException e) {
        // there is nothing to be done here because it is not
        // expected that the odometry correction will be
        // interrupted by another thread
      }
    }

    return hasLine;
  }

  public float getReading() {
    colorSensor.fetchSample(colorSample, 0);

    return colorSample[0] * 100;
  }

  /**
   * This method returns a looped counter
   * 
   * @param offSet By how much the current counter is offset
   * @return the looped value
   */
  public int getFilterPos(int offSet) {
    int nextValue = filterPos + offSet;
    if (nextValue < 0) {
      nextValue += valueSize;
    }
    return nextValue % valueSize;
  }

  /**
   * Calibrates the sensor
   */
  public void calibrateSensor() {
    // Does a few readings so that the filtering has enough values to recognize line changes
    for (int i = 0; i < 2 * calibDelay; i++) {
      hasLine();
    }
  }

  /**
   * Returns the color sensor used by this object
   * 
   * @return the EV3ColorSensor object
   */
  public EV3ColorSensor getColorSensor() {
    return sensor;
  }
}
