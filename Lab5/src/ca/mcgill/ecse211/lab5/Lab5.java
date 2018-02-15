// Lab2.java

package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

  // INPUTS FED (test)
  public static int startingPosition = 1; // starting corner
  public static int Xo = 1; // position to get to
  public static int Yo = 6;
  public static int Xc = 2; // zipline start (need to face this point)
  public static int Yc = 6;

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  public static final double WHEEL_RADIUS = 2.1; // Radius of the wheels of our car
  public static final double TRACK = 15; // track value of our robot
  public final static double TILE_DIST = 30.48;

  private static final Port usPort = LocalEV3.get().getPort("S1");


  public static void main(String[] args) {

    final TextLCD t = LocalEV3.get().getTextLCD();

    InputUI.setInputCoordinates();

    Odometer odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);


    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
                                                         // returned
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);

    Navigator navigator = new Navigator(odometer, leftMotor, rightMotor, usPoller);


    odometer.start(); // start threads and set path to the waypoints
    odometryDisplay.start();
    USLocalizer usLocalizer = new USLocalizer(odometer, usPoller,
        USLocalizer.LocalizerType.FALLING_EDGE, leftMotor, rightMotor, navigator);
    usLocalizer.doLocalization();

    LightLocalizer lightLocalizer =
        new LightLocalizer(odometer, usPoller, leftMotor, rightMotor, navigator);
    lightLocalizer.doLocalizationOnly();

    navigator.travelTo(30.48 * PathGenerator.getStartX(startingPosition),
        30.48 * PathGenerator.getStartY(startingPosition));

    int[] path = PathGenerator.generatePath(Xc, Yc, Xo, Yo, startingPosition);

    navigator.setPath(path);
    wait(1000);

    navigator.travelToZip();
    lightLocalizer.doLocalizationOnly();

    navigator.travelTo(Xo * Lab5.TILE_DIST, Yo * Lab5.TILE_DIST);


    navigator.turnTo(0);

    while (Button.waitForAnyPress() == Button.ID_ALL);

    navigator.travelTo(Xo * Lab5.TILE_DIST, Yo * Lab5.TILE_DIST - 3);
    navigator.turnTo(0);

    navigator.mountZip();



    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }



  public static void wait(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
