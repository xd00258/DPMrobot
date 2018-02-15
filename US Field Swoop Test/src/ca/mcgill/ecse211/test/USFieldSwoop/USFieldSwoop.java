package ca.mcgill.ecse211.test.USFieldSwoop;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USFieldSwoop {
  public static final double TRACK = 15;
  public static final double TILE_DIST = 30.48;
  public static final double WHEEL_RADIS = 2.1;

  Navigator navigator;

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  private static final Port usPort = LocalEV3.get().getPort("S1");

  public static void main(String args[]) {
    Odometer odometer = new Odometer(leftMotor, rightMotor);

    //TODO should also test different speeds
    Navigator navigator = new Navigator(leftMotor, rightMotor, 50);

    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance");

    float[] usData = new float[usDistance.sampleSize()];
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, odometer, true);
    System.out.println("Please press on any button to contiue...");
    while (Button.waitForAnyPress() == Button.ID_ALL);
    
    odometer.start();
    usPoller.start();
    navigator.rotate(-90);
    usPoller.done = true;
  }
}
