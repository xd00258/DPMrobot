package ca.mcgill.ecse211.lab5;

public class PathGenerator {    

  /** Generates a path based on starting position, position, orientation of zipline */
  public static int[] generatePath(int x_c, int y_c, int x_0, int y_0, int startingPos) {

    // Determine starting position in x and y coordinates
    int start_x = getStartX(startingPos);
    int start_y = getStartY(startingPos);

    // Determine the change of position
    int dX = x_0 - start_x;
    int dY = y_0 - start_y;

    // Path array that will be returned
    int[] path = new int[4];

    // Robot will always follow a safe corridor
    // These if statements makes sure it always does the largest displacement first
    // Thus, this ensures it is always avoiding the zipline
    if (Math.abs(dX) > Math.abs(dY)) {
      // Do x displacement first
      path[0] = start_x + dX;
      path[1] = start_y;

      // Then y displacement
      path[2] = start_x + dX;
      path[3] = start_y + dY;
    } else if (Math.abs(dX) < Math.abs(dY)) {
      // Do y displacement first
      path[0] = start_x;
      path[1] = start_y + dY;

      // Then x displacement
      path[2] = start_x + dX;
      path[3] = start_y + dY;
    } else { // If both distances are equal (highly unlikely but implemented just in case)
      // These represent the "orientation" of the zipline
      int orX = x_c - x_0;
      int orY = y_c - y_0;

      // If the zipline is on the x-axis
      if (orX != 0 && orY == 0) {
        // Do x displacement first
        path[0] = start_x + dX;
        path[1] = start_y;

        // Then y displacement
        path[2] = start_x + dX;
        path[3] = start_y + dY;
      } else if (orX == 0 && orY != 0) { // If the zipline is on the y-axis
        // Do y displacement first
        path[0] = start_x;
        path[1] = start_y + dY;

        // Then x displacement
        path[2] = start_x + dX;
        path[3] = start_y + dY;
      } else { // This is only possible if zipline coordinates were improperly inputted
        // Returns null to indicate error
        return null;
      }
    }

    return path;
  }

  /** Return the x coordinate based on starting position */
  public static int getStartX(int pos) {
    switch (pos) {
      case 0:
        return 1;
      case 1:
        return 7;
      case 2:
        return 7;
      case 3:
        return 1;
      default:
        return -1;
    }
  }

  /** Returns the y coordinate based on starting position */
  public static int getStartY(int pos) {
    switch (pos) {
      case 0:
        return 1;
      case 1:
        return 1;
      case 2:
        return 7;
      case 3:
        return 7;
      default:
        return -1;
    }
  }
}
