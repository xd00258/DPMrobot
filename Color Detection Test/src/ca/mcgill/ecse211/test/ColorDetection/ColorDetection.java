package ca.mcgill.ecse211.test.ColorDetection;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class ColorDetection {
  public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
  
  public static void main(String args[]) {
    System.out.println("Hello this is the first one");
    while (true) {
      int color = colorSensor.getColorID();
      System.out.println(convertColor(color));
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }
  
  public static String convertColor(int color) {
    switch (color) {
      case 0: return "Red";
      case 1: return "Green";
      case 2: return "Blue";
      case 3: return "Yellow";
      case 4: return "Magenta";
      case 5: return "Orange";
      case 6: return "White";
      case 7: return "Black";
      case 8: return "Pink";
      case 9: return "Gray";
      case 10: return "Light Gray";
      case 11: return "Dark Gray";
      case 12: return "Cyan";
      case 13: return "Brown";
      default: return "Unrecognizable";
    }
  }
}
