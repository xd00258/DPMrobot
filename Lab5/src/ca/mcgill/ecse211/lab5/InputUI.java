package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

public class InputUI {
  static int active = 1;

  public static void setInputCoordinates() {
    int buttonChoice;

    final TextLCD t = LocalEV3.get().getTextLCD();

    do {
      // clear the display
      t.clear();

      // left will decrement the value, right will increment it
      t.drawString("Left-- | Right++", 0, 0);

      // change the display depending on which element is being changed
      switch (active) {
        case 1:
          t.drawString(">X0: " + Lab5.Xo + "    Y0: " + Lab5.Yo + " ", 0, 1);
          t.drawString("XC: " + Lab5.Xc + "     YC: " + Lab5.Yc + " ", 0, 2);
          t.drawString("Corner: " + Lab5.startingPosition + "       ", 0, 3);
          break;
        case 2:
          t.drawString("X0: " + Lab5.Xo + "    >Y0: " + Lab5.Yo + " ", 0, 1);
          t.drawString("XC: " + Lab5.Xc + "     YC: " + Lab5.Yc + " ", 0, 2);
          t.drawString("Corner: " + Lab5.startingPosition + "       ", 0, 3);
          break;
        case 3:
          t.drawString("X0: " + Lab5.Xo + "     Y0: " + Lab5.Yo + " ", 0, 1);
          t.drawString(">XC: " + Lab5.Xc + "    YC: " + Lab5.Yc + " ", 0, 2);
          t.drawString("Corner: " + Lab5.startingPosition + "       ", 0, 3);
          break;
        case 4:
          t.drawString("X0: " + Lab5.Xo + "     Y0: " + Lab5.Yo + " ", 0, 1);
          t.drawString("XC: " + Lab5.Xc + "    >YC: " + Lab5.Yc + " ", 0, 2);
          t.drawString("Corner: " + Lab5.startingPosition + "       ", 0, 3);
          break;
        case 5:
          t.drawString("X0: " + Lab5.Xo + "     Y0: " + Lab5.Yo + " ", 0, 1);
          t.drawString("XC: " + Lab5.Xc + "     YC: " + Lab5.Yc + " ", 0, 2);
          t.drawString(">Corner: " + Lab5.startingPosition + "      ", 0, 3);
          break;
      }

      buttonChoice = Button.waitForAnyPress();

      if (buttonChoice == Button.ID_LEFT) { // decrement the value of the active variable
        switch (active) {
          case 1:
            if (Lab5.Xo == 0) { // Ensure it stays in bounds
              Lab5.Xo = 8;
            } else {
              Lab5.Xo--;
            }
            break;
          case 2:
            if (Lab5.Yo == 0) {
              Lab5.Yo = 8;
            } else {
              Lab5.Yo--;
            }
            break;
          case 3:
            if (Lab5.Xc == 0) {
              Lab5.Xc = 8;
            } else {
              Lab5.Xc--;
            }
            break;
          case 4:
            if (Lab5.Yc == 0) {
              Lab5.Yc = 8;
            } else {
              Lab5.Yc--;
            }
            break;
          case 5:
            if (Lab5.startingPosition == 0) {
              Lab5.startingPosition = 3;
            } else {
              Lab5.startingPosition--;
            }
            break;
        }
      } else if (buttonChoice == Button.ID_RIGHT) { // increment the value of the active variable
        switch (active) {
          case 1:
            if (Lab5.Xo == 8) {
              Lab5.Xo = 0;
            } else {
              Lab5.Xo++;
            }
            break;
          case 2:
            if (Lab5.Yo == 8) {
              Lab5.Yo = 0;
            } else {
              Lab5.Yo++;
            }
            break;
          case 3:
            if (Lab5.Xc == 8) {
              Lab5.Xc = 0;
            } else {
              Lab5.Xc++;
            }
            break;
          case 4:
            if (Lab5.Yc == 8) {
              Lab5.Yc = 0;
            } else {
              Lab5.Yc++;
            }
            break;
          case 5:
            if (Lab5.startingPosition == 3) {
              Lab5.startingPosition = 0;
            } else {
              Lab5.startingPosition++;
            }
            break;
        }
      } else if (buttonChoice == Button.ID_ENTER) { // Go to next element
        active++;
      } else if (buttonChoice == Button.ID_ESCAPE) { // Go to previous element
        if (active > 1) {
          active--;
        }
      }
    } while (active < 6); // Break out of loop once all values are set
  }
}
