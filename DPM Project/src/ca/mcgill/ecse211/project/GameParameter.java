package ca.mcgill.ecse211.project;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * The GameParameter class holds all the game parameters provided by the game server.
 *
 * @author Evan Laflamme
 */

public class GameParameter {
  /** Team starting from red zone (1-20) */
  public int RedTeam = 1;
  /** Team starting from green zone (1-20) */
  public int GreenTeam = 1;
  /** Starting corner for red team (1-4) */
  public int RedCorner = 1;
  /** Starting corner for green team (1-4) */
  public int GreenCorner = 0;
  /** Color of green opponent flag (1-5) */
  public int OG = 1;
  /** Color of red opponent flag (1-5) */
  public int OR = 1;

  /** X coordinate of lower left hand corner of Red Zone */
  public int Red_LL_x = 0;
  /** Y coordinate of lower left hand corner of Red Zone */
  public int Red_LL_y = 0;
  /** X coordinate of upper right hand corner of Red Zone */
  public int Red_UR_x = 0;
  /** Y coordinate of upper right hand corner of Red Zone */
  public int Red_UR_y = 0;

  /** X coordinate of lower left hand corner of Green Zone */
  public int Green_LL_x = 0;
  /** Y coordinate of lower left hand corner of Green Zone */
  public int Green_LL_y = 0;
  /** X coordinate of upper right hand corner of Green Zone */
  public int Green_UR_x = 0;
  /** Y coordinate of upper right hand corner of Green Zone */
  public int Green_UR_y = 0;

  /** X coordinate of end point corresponding to zip line in Red Zone */
  public int ZC_R_x = 0;
  /** Y coordinate of end point corresponding to zip line in Red Zone */
  public int ZC_R_y = 0;
  /** X coordinate of end point together with ZC_R indicates direction of zip line */
  public int ZO_R_x = 0;
  /** Y coordinate of end point together with ZC_R indicates direction of zip line */
  public int ZO_R_y = 0;

  /** X coordinate of end point corresponding to zip line in Green Zone */
  public int ZC_G_x = 2;
  /** Y coordinate of end point corresponding to zip line in Green Zone */
  public int ZC_G_y = 6;
  /** X coordinate of end point together with ZC_G indicates direction of zip line */
  public int ZO_G_x = 1;
  /** Y coordinate of end point together with ZC_G indicates direction of zip line */
  public int ZO_G_y = 6;

  /** X coordinate of lower left hand corner of horizontal shallow water zone */
  public int SH_LL_x = 0;
  /** Y coordinate of lower left hand corner of horizontal shallow water zone */
  public int SH_LL_y = 0;
  /** X coordinate of upper right hand corner of horizontal shallow water zone */
  public int SH_UR_x = 0;
  /** Y coordinate of upper right hand corner of horizontal shallow water zone */
  public int SH_UR_y = 0;

  /** X coordinate of lower left hand corner of vertical shallow water zone */
  public int SV_LL_x = 0;
  /** Y coordinate of lower left hand corner of vertical shallow water zone */
  public int SV_LL_y = 0;
  /** X coordinate of upper right hand corner of vertical shallow water zone */
  public int SV_UR_x = 0;
  /** Y coordinate of upper right hand corner of vertical shallow water zone */
  public int SV_UR_y = 0;

  /** X coordinate of lower left hand corner of search region in red player zone */
  public int SR_LL_x = 0;
  /** Y coordinate of lower left hand corner of search region in red player zone */
  public int SR_LL_y = 0;
  /** X coordinate of upper right hand corner of search region in red player zone */
  public int SR_UR_x = 0;
  /** Y coordinate of upper right hand corner of search region in red player zone */
  public int SR_UR_y = 0;

  /** X coordinate of lower left hand corner of search region in green player zone */
  public int SG_LL_x = 0;
  /** Y coordinate of lower left hand corner of search region in green player zone */
  public int SG_LL_y = 0;
  /** X coordinate of upper right hand corner of search region in green player zone */
  public int SG_UR_x = 0;
  /** Y coordinate of upper right hand corner of search region in green player zone */
  public int SG_UR_y = 0;

  /**
   * GameParameter constructor.
   */
  public GameParameter() {
    setGameParametersFromServer();
  }

  /**
   * This method will connect to the game server and set all game parameters accordingly.
   */
  @SuppressWarnings("rawtypes")
  private void setGameParametersFromServer() {
    WifiConnection conn = new WifiConnection(DPMProject.SERVER_IP, DPMProject.TEAM_NUMBER, false);
    boolean wasSet = false;

    while (!wasSet) {
      // Connect to server and get the data, catching any errors that might occur
      try {
        Map data = conn.getData(); // Get all data from server

        // Set game parameters to server values
        RedTeam = ((Long) data.get("RedTeam")).intValue();
        GreenTeam = ((Long) data.get("GreenTeam")).intValue();
        RedCorner = ((Long) data.get("RedCorner")).intValue();
        GreenCorner = ((Long) data.get("GreenCorner")).intValue();
        OG = ((Long) data.get("OG")).intValue();
        OR = ((Long) data.get("OR")).intValue();

        Red_LL_x = ((Long) data.get("Red_LL_x")).intValue();
        Red_LL_y = ((Long) data.get("Red_LL_y")).intValue();
        Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
        Red_UR_y = ((Long) data.get("Red_UR_y")).intValue();

        Green_LL_x = ((Long) data.get("Green_LL_x")).intValue();
        Green_LL_y = ((Long) data.get("Green_LL_y")).intValue();
        Green_UR_x = ((Long) data.get("Green_UR_x")).intValue();
        Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();

        ZC_R_x = ((Long) data.get("ZC_R_x")).intValue();
        ZC_R_y = ((Long) data.get("ZC_R_y")).intValue();
        ZO_R_x = ((Long) data.get("ZO_R_x")).intValue();
        ZO_R_y = ((Long) data.get("ZO_R_y")).intValue();

        ZC_G_x = ((Long) data.get("ZC_G_x")).intValue();
        ZC_G_y = ((Long) data.get("ZC_G_y")).intValue();
        ZO_G_x = ((Long) data.get("ZO_G_x")).intValue();
        ZO_G_y = ((Long) data.get("ZO_G_y")).intValue();

        SH_LL_x = ((Long) data.get("SH_LL_x")).intValue();
        SH_LL_y = ((Long) data.get("SH_LL_y")).intValue();
        SH_UR_x = ((Long) data.get("SH_UR_x")).intValue();
        SH_UR_y = ((Long) data.get("SH_UR_y")).intValue();

        SV_LL_x = ((Long) data.get("SV_LL_x")).intValue();
        SV_LL_y = ((Long) data.get("SV_LL_y")).intValue();
        SV_UR_x = ((Long) data.get("SV_UR_x")).intValue();
        SV_UR_y = ((Long) data.get("SV_UR_y")).intValue();

        SR_LL_x = ((Long) data.get("SR_LL_x")).intValue();
        SR_LL_y = ((Long) data.get("SR_LL_y")).intValue();
        SR_UR_x = ((Long) data.get("SR_UR_x")).intValue();
        SR_UR_y = ((Long) data.get("SR_UR_y")).intValue();

        SG_LL_x = ((Long) data.get("SG_LL_x")).intValue();
        SG_LL_y = ((Long) data.get("SG_LL_y")).intValue();
        SG_UR_x = ((Long) data.get("SG_UR_x")).intValue();
        SG_UR_y = ((Long) data.get("SG_UR_y")).intValue();

        wasSet = true;

      } catch (Exception e) {
        wasSet = false;
      }
    }
  }
}

