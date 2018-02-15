package ca.mcgill.ecse211.test.USFieldSwoop;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
