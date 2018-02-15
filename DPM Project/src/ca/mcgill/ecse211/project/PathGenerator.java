package ca.mcgill.ecse211.project;

import java.util.ArrayList;
import java.util.Collections;

/**
 * This class is used to generate paths for the robot to follow
 * 
 * @author Karl Godin, Evan Laflamme
 *
 */
public class PathGenerator {
  /** A reference to the game parameters */
  private GameParameter gameParams;
  /** A reference to the robot's odometer */
  private Odometer odometer;

  /** A distance threshold indicating how many tiles the robot should travel before relocalizing */
  private final int distanceThreshold = 3;

  /**
   * PathGenerator constructor.
   * 
   * @param gameParams a reference to the game parameters
   * @param odometer a reference to the robot's odometer
   */
  public PathGenerator(GameParameter gameParams, Odometer odometer) {
    this.gameParams = gameParams;
    this.odometer = odometer;
  }

  /**
   * This method will generate the path to travel across the shallow region of the river. It will
   * ensure that the robot does not hit any known obstacle along the way. This method will take into
   * account which team the robot is on in order to know whether the robot is going to or coming
   * from their zone in order to travel the appropriate path.
   * 
   * @return an ArrayList containing the points along the path
   */
  public ArrayList<double[]> generateRiverPath() {
    // The corner point in the river path
    double[] corner = new double[] {(gameParams.SV_LL_x + gameParams.SV_UR_x) / 2.0,
        (gameParams.SH_LL_y + gameParams.SH_UR_y) / 2.0, 2};

    int startX = Math.round((float) (odometer.getX() / DPMProject.TILE_LENGTH)); // Get closest
                                                                                 // point to robot
    int startY = Math.round((float) (odometer.getY() / DPMProject.TILE_LENGTH));

    ArrayList<double[]> path = new ArrayList<>();

    if ((gameParams.SH_UR_x == gameParams.SV_UR_x) && (gameParams.SH_UR_y == gameParams.SV_UR_y)
        && (gameParams.SH_LL_y != gameParams.SV_LL_y)) { // Case 1 and 2
      path.add(new double[] {gameParams.SV_LL_x, gameParams.SV_LL_y - 1, 1}); // Go to closest point
                                                                              // to zipline

      path.add(new double[] {corner[0], gameParams.SV_LL_y, 0}); // Go directly in front of
                                                                 // horizontal bit

      path.add(corner); // Travel to corner

      path.add(new double[] {gameParams.SH_LL_x, corner[1], 0}); // Go directly in front of green
                                                                 // zone

      path.add(new double[] {gameParams.SH_LL_x - 1, gameParams.SH_LL_y, 1}); // Go to closet point
                                                                              // after crossing

      if (gameParams.Red_LL_y > gameParams.Green_UR_y) { // Going in reverse direction
        Collections.reverse(path);
      }
    } else if ((gameParams.SV_UR_y == gameParams.SH_UR_y)
        && (gameParams.SV_UR_x != gameParams.SH_UR_x)) { // Case 3 and 4
      path.add(new double[] {gameParams.SV_LL_x, gameParams.SV_LL_y - 1, 1}); // Go to closest point
                                                                              // to zipline

      path.add(new double[] {corner[0], gameParams.SV_LL_y, 2}); // Go directly in front of vertical
                                                                 // bit

      path.add(corner); // Travel to corner

      path.add(new double[] {gameParams.SH_UR_x, corner[1], 0}); // Go directly in front of green
                                                                 // zone

      path.add(new double[] {gameParams.SH_UR_x + 1, gameParams.SH_UR_y, 1}); // Go to closet point
                                                                              // after crossing

      if (gameParams.Red_LL_y > gameParams.Green_UR_y) { // Going in reverse direction
        Collections.reverse(path);
      }
    } else if ((gameParams.SV_LL_x == gameParams.SH_LL_x)
        && (gameParams.SV_LL_y == gameParams.SH_LL_y)
        && (gameParams.SH_UR_y != gameParams.SV_UR_y)) { // Case 5 and 6
      path.add(new double[] {gameParams.SH_UR_x + 1, gameParams.SH_UR_y, 1}); // Go to closest point
                                                                              // to zipline

      path.add(new double[] {gameParams.SH_UR_x, corner[1], 0}); // Go directly in front of
                                                                 // horizontal bit

      path.add(corner); // Travel to corner

      path.add(new double[] {corner[0], gameParams.SV_UR_y, 0}); // Go directly in front of green
                                                                 // zone

      path.add(new double[] {gameParams.SV_UR_x, gameParams.SV_UR_y + 1, 1}); // Go to closet point
                                                                              // after crossing

      if (gameParams.Red_LL_y > gameParams.Green_UR_y) { // Going in reverse direction
        Collections.reverse(path);
      }
    } else if ((gameParams.SV_UR_x == gameParams.SH_UR_x)
        && (gameParams.SV_UR_y != gameParams.SH_UR_y)) { // Case 7 and 8
      path.add(new double[] {gameParams.SH_LL_x - 1, gameParams.SH_LL_y, 1}); // Go to closest point
                                                                              // to zipline

      path.add(new double[] {gameParams.SH_LL_x, corner[1], 0}); // Go directly in front of
                                                                 // horizontal bit

      path.add(corner); // Travel to corner

      path.add(new double[] {corner[0], gameParams.SV_UR_y, 0}); // Go directly in front of green
                                                                 // zone

      path.add(new double[] {gameParams.SV_UR_x, gameParams.SV_UR_y + 1, 1}); // Go to closet point
                                                                              // after crossing

      if (gameParams.Red_LL_y > gameParams.Green_UR_y) { // Going in reverse direction
        Collections.reverse(path);
      }
    } else if ((gameParams.SH_UR_y - 1 == gameParams.SH_LL_y)
        && (gameParams.SV_UR_y - 1 == gameParams.SV_LL_y)) { // Case 9 and 10
      path.add(new double[] {gameParams.SH_UR_x + 1, gameParams.SH_UR_y, 1}); // Go to closest point
                                                                              // to zipline

      path.add(new double[] {gameParams.SH_UR_x, corner[1], 0}); // Go directly in front of
                                                                 // horizontal bit

      path.add(new double[] {gameParams.SH_LL_x, corner[1], 0}); // Go directly in front of green
                                                                 // zone

      path.add(new double[] {gameParams.SH_LL_x - 1, gameParams.SH_LL_y, 1}); // Go to closet point
                                                                              // after crossing

      if (gameParams.Red_UR_x < gameParams.Green_LL_x) { // Going in reverse direction
        Collections.reverse(path);
      }
    } else { // Case 11 and 12
      path.add(new double[] {gameParams.SV_UR_x, gameParams.SV_UR_y + 1, 1}); // Go to closest point
                                                                              // to zipline

      path.add(new double[] {corner[0], gameParams.SV_UR_y, 0}); // Go directly in front of
                                                                 // horizontal bit

      path.add(new double[] {corner[0], gameParams.SV_LL_y, 0}); // Go directly in front of green
                                                                 // zone

      path.add(new double[] {gameParams.SV_LL_x, gameParams.SH_LL_y - 1, 1}); // Go to closet point
                                                                              // after crossing

      if (gameParams.Red_UR_y < gameParams.Green_UR_y) { // Going in reverse direction
        Collections.reverse(path);
      }
    }

    path.get(1)[2] = 2; // Wiggle at closest horizontal or vertical bit

    // Get closest point to river crossing
    int x = (int) path.get(0)[0];
    int y = (int) path.get(0)[1];

    int dX = x - startX;
    int dY = y - startY;

    ArrayList<double[]> initialPath = generateRegularPath(startX, startY, dX, dY); // Generate path
                                                                                   // to closest
                                                                                   // river point

    initialPath.remove(initialPath.size() - 1); // Remove last point from initial path (corresponds
                                                // to first point in second path)
    initialPath.addAll(path); // Combine the two paths

    initialPath.get(initialPath.size() - 1)[2] = 0; // Don't localize at the end of the path


    return initialPath;
  }

  /**
   * This method will generate the path to travel to the zipline. It will ensure that we are not
   * going to hit any known obstacle along the way. This method will take into account which team
   * the robot is on in order to know whether the robot is going to or coming from their zone in
   * order to travel the appropriate path.
   * 
   * @return an ArrayList containing the points along the path
   */
  public ArrayList<double[]> generateZiplinePath() {
    // Either team will mount the zipline from the within the green zone
    // so the x0 and y0 will be the same for either team
    double x0 = gameParams.ZO_G_x;
    double y0 = gameParams.ZO_G_y;

    double startX = 0;
    double startY = 0;

    // Determine starting position in x and y coordinates
    // Red team will have just localized after finding the flag
    // Green team will be starting from original corner
    if (gameParams.RedTeam == DPMProject.TEAM_NUMBER) {
      startX = Math.round((float) (odometer.getX() / DPMProject.TILE_LENGTH)); // Get closest point
                                                                               // to robot
      startY = Math.round((float) (odometer.getY() / DPMProject.TILE_LENGTH)); // Get closest point
                                                                               // to robot
    } else if (gameParams.GreenTeam == DPMProject.TEAM_NUMBER) {
      startX = getStartX(DPMProject.startingPosition);
      startY = getStartY(DPMProject.startingPosition);
    }

    // Determine the change of position
    double dX = x0 - startX;
    double dY = y0 - startY;

    return generateRegularPath(startX, startY, dX, dY);
  }

  /**
   * This method will generate the path to the "flag" zone from the current location depending on
   * the team of the robot.
   * 
   * @return an ArrayList containing the points along the path
   */
  public ArrayList<double[]> generateFlagZonePath() {
    int startX = Math.round((float) (odometer.getX() / DPMProject.TILE_LENGTH)); // Get closest
                                                                                 // point to robot
    int startY = Math.round((float) (odometer.getY() / DPMProject.TILE_LENGTH));

    int x0 = 0;
    int y0 = 0;

    int zone_LL_x = 0;
    int zone_LL_y = 0;
    int zone_UR_x = 0;
    int zone_UR_y = 0;

    // Set flag region bounds based on team
    if (gameParams.RedTeam == DPMProject.TEAM_NUMBER) {
      zone_LL_x = gameParams.SG_LL_x;
      zone_LL_y = gameParams.SG_LL_y;
      zone_UR_x = gameParams.SG_UR_x;
      zone_UR_y = gameParams.SG_UR_y;

    } else if (gameParams.GreenTeam == DPMProject.TEAM_NUMBER) {
      zone_LL_x = gameParams.SR_LL_x;
      zone_LL_y = gameParams.SR_LL_y;
      zone_UR_x = gameParams.SR_UR_x;
      zone_UR_y = gameParams.SR_UR_y;
    }


    // Determine closest corner of flag zone and set that as target
    int dX_LL = Math.abs(zone_LL_x - startX);
    int dY_LL = Math.abs(zone_LL_y - startY);
    int dX_UR = Math.abs(zone_UR_x - startX);
    int dY_UR = Math.abs(zone_UR_y - startY);

    if (dX_LL > dX_UR) {
      x0 = zone_UR_x;
    } else if (dX_LL < dX_UR) {
      x0 = zone_LL_x;
    } else {
      x0 = zone_LL_x;
    }

    if (dY_LL > dY_UR) {
      y0 = zone_UR_y;
    } else if (dY_LL < dX_UR) {
      y0 = zone_LL_y;
    } else {
      y0 = zone_LL_y;
    }

    // Determine the change of position
    int dX = x0 - startX;
    int dY = y0 - startY;

    return generateRegularPath(startX, startY, dX, dY);
  }

  /**
   * This method will generate the path to the original corner the robot started in from its current
   * position.
   * 
   * @return an ArrayList containing the points along the path
   */
  public ArrayList<double[]> generateHomePath() {
    int startX = (int) (odometer.getX() / DPMProject.TILE_LENGTH); // Get closest point to robot
    int startY = (int) (odometer.getY() / DPMProject.TILE_LENGTH);

    int x0 = getStartX(DPMProject.startingPosition);
    int y0 = getStartY(DPMProject.startingPosition);

    // Determine the change of position
    int dX = x0 - startX;
    int dY = y0 - startY;

    ArrayList<double[]> path = new ArrayList<>();

    // Robot will always follow a safe corridor
    // These if statements makes sure it always does the largest displacement first
    if (dX == 0) {
      // Then only y displacement
      path.add(new double[] {startX, startY + dY, 0});
    } else if (dY == 0) {
      // Then only x displacement
      path.add(new double[] {startX + dX, startY, 0});
    } else {
      int zone_LL_x = 0;
      int zone_LL_y = 0;
      int zone_UR_x = 0;
      int zone_UR_y = 0;

      // Set flag region bounds based on team
      if (gameParams.RedTeam == DPMProject.TEAM_NUMBER) {
        zone_LL_x = gameParams.SR_LL_x;
        zone_LL_y = gameParams.SR_LL_y;
        zone_UR_x = gameParams.SR_UR_x;
        zone_UR_y = gameParams.SR_UR_y;

      } else if (gameParams.GreenTeam == DPMProject.TEAM_NUMBER) {
        zone_LL_x = gameParams.SG_LL_x;
        zone_LL_y = gameParams.SG_LL_y;
        zone_UR_x = gameParams.SG_UR_x;
        zone_UR_y = gameParams.SG_UR_y;
      }

      if ((((startX + dX) <= zone_LL_x) && (startY <= zone_LL_y)
          || ((startX + dX) >= zone_UR_x) && (startY >= zone_UR_y))) { // If x displacement first
                                                                       // doesn't go through flag
                                                                       // region
        // Do x displacement first
        path.add(new double[] {startX + dX, startY, 0});

        // Then y displacement
        path.add(new double[] {startX + dX, startY + dY, 0});
      } else if (((startX <= zone_LL_x) && ((startY + dY) <= zone_LL_y))
          || ((startX >= zone_UR_x) && ((startY + dY) >= zone_UR_y))) { // If y displacement first
                                                                        // doesn't go through flag
                                                                        // region
        // Do y displacement first
        path.add(new double[] {startX, startY + dY, 0});

        // Then x displacement
        path.add(new double[] {startX + dX, startY + dY, 0});
      } else {
        // Do x displacement first
        path.add(new double[] {startX + dX, startY, 0});

        // Then y displacement
        path.add(new double[] {startX + dX, startY + dY, 0});
      }
    }

    return path;
  }

  /**
   * This method will generate a regular path given the input parameters. In our case, a regular
   * path is a path that takes a single x and/or a single y displacement.
   * 
   * @param startX the x coordinate of the starting location
   * @param startY the y coordinate of the starting location
   * @param dX the x displacement
   * @param dY the y displacement
   * 
   * @return an ArrayList containing the points along the path
   */
  private ArrayList<double[]> generateRegularPath(double startX, double startY, double dX,
      double dY) {
    ArrayList<double[]> path = new ArrayList<>();

    if (Math.abs(dX) < 1) {
      // Then only y displacement
      if (Math.abs(dY) > distanceThreshold) {
        double nextY = startY + Math.round(dY / 2);
        nextY = (nextY % 4 == 0) ? (nextY + 1) : nextY;

        path.add(new double[] {startX, nextY, 1});
      }

      path.add(new double[] {startX, startY + dY, 0});
    } else if (Math.abs(dY) < 1) {
      // Then only x displacement
      if (Math.abs(dX) > distanceThreshold) {
        double nextX = startX + Math.round(dX / 2);
        nextX = (nextX % 4 == 0) ? (nextX + 1) : nextX;

        path.add(new double[] {nextX, startY, 1});
      }

      path.add(new double[] {startX + dX, startY, 0});
    } else if (Math.abs(dX) > Math.abs(dY)) {
      // Do x displacement first, break path down
      if (Math.abs(dX) > distanceThreshold) {
        double nextX = startX + Math.round(dX / 2);
        nextX = (nextX % 4 == 0) ? (nextX + 1) : nextX;

        path.add(new double[] {nextX, startY, 1});
      }

      path.add(new double[] {startX + dX, startY, 1});

      // Then y displacement
      if (Math.abs(dY) > distanceThreshold) {
        double nextY = startY + Math.round(dY / 2);
        nextY = (nextY % 4 == 0) ? (nextY + 1) : nextY;

        path.add(new double[] {startX + dX, nextY, 1});
      }

      path.add(new double[] {startX + dX, startY + dY, 0});

    } else if (Math.abs(dX) < Math.abs(dY)) {
      // Do y displacement first
      if (Math.abs(dY) > distanceThreshold) {
        double nextY = startY + Math.round(dY / 2);
        nextY = (nextY % 4 == 0) ? (nextY + 1) : nextY;

        path.add(new double[] {startX, nextY, 1});
      }

      path.add(new double[] {startX, startY + dY, 1});

      // Then x displacement
      if (Math.abs(dX) > distanceThreshold) {
        double nextX = startX + Math.round(dX / 2);
        nextX = (nextX % 4 == 0) ? (nextX + 1) : nextX;

        path.add(new double[] {nextX, startY + dY, 1});
      }

      path.add(new double[] {startX + dX, startY + dY, 0});
    } else { // Both distances equal, so just go x displacement first
      // Do x displacement first, break path down
      if (Math.abs(dX) > distanceThreshold) {
        double nextX = startX + Math.round(dX / 2);
        nextX = (nextX % 4 == 0) ? (nextX + 1) : nextX;

        path.add(new double[] {nextX, startY, 1});
      }

      path.add(new double[] {startX + dX, startY, 1});

      // Then y displacement
      if (Math.abs(dY) > distanceThreshold) {
        double nextY = startY + Math.round(dY / 2);
        nextY = (nextY % 4 == 0) ? (nextY + 1) : nextY;

        path.add(new double[] {startX + dX, nextY, 1});
      }

      path.add(new double[] {startX + dX, startY + dY, 0});
    }

    return path;
  }

  /**
   * Returns the x coordinate based on starting position
   * 
   * @param pos the pos parameter is an int from 0 to 3 representing the starting corner
   * @return x coordinate
   */
  public static int getStartX(int pos) {
    switch (pos) {
      case 0:
        return 1;
      case 1:
        return DPMProject.BOARD_LENGTH - 1;
      case 2:
        return DPMProject.BOARD_LENGTH - 1;
      case 3:
        return 1;
      default:
        return -1;
    }
  }

  /**
   * Returns the y coordinate based on starting position
   * 
   * @param pos the pos parameter is an int from 0 to 3 representing the starting corner
   * @return y coordinate
   */
  public static int getStartY(int pos) {
    switch (pos) {
      case 0:
        return 1;
      case 1:
        return 1;
      case 2:
        return DPMProject.BOARD_LENGTH - 1;
      case 3:
        return DPMProject.BOARD_LENGTH - 1;
      default:
        return -1;
    }
  }
}
