package missdaisy.test;

import java.util.Scanner;

import missdaisy.autonomous.AutonomousParser;
import missdaisy.autonomous.StateMachine;
import missdaisy.fileio.PropertyReader;

/**
 * This class is designed to help test the autonomous mode selector.
 *
 * @author joshs
 */
public class AutoModeSelector {

  private static final String kAutonomousFilePath =
      System.getProperty("user.home") + "\\Desktop\\Autonomous_Modes\\";
  private static boolean mIsRedSide = true;
  private static int mNumAutoPos = 3;
  private static int mAutoPos = 2;
  private static boolean mGoForAutoLoader = false;
  private static String mAutoMode;
  private static String selector = "";
  private static Scanner in = new Scanner(System.in);
  private static PropertyReader mPropertyReader = new PropertyReader();

  public static void main(String[] args) {
    System.out.println("Autonomous file path: " + kAutonomousFilePath);
    chooseAutoMode(true, mAutoPos, false);
    selector = in.nextLine();
    while (true) {
      if (selector.equals("p")) {
        mAutoPos++;
        if (mAutoPos > mNumAutoPos) {
          mAutoPos = 2;
        }
        chooseAutoMode(mIsRedSide, mAutoPos, mGoForAutoLoader);
      } else if (selector.equals("d")) {
        mAutoPos++;
        if (mAutoPos > mNumAutoPos) {
          mAutoPos = 1;
        }
        chooseAutoMode(mIsRedSide, mAutoPos, mGoForAutoLoader);
      } else if (selector.equals("r")) {
        StateMachine machine =
            new StateMachine(new AutonomousParser().parseStates(true));
        while (true) {
          machine.run();
          try {
            Thread.sleep(19);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
      selector = in.nextLine();
    }
  }

  public static void chooseAutoMode(boolean mIsRedSide, int mStartPosition,
      boolean mGoForAutoLoader) {
    if (mIsRedSide) {
      mAutoMode = "Red Side";
    } else {
      mAutoMode = "Blue Side";
    }
    System.out.println(mAutoMode);

    switch (mStartPosition) {
      case 6:
        mAutoMode = "Cross";
        break;
      case 7:
        mAutoMode = "Do Nothing";
        break;
      case 8:
        mAutoMode = "Reach";
        break;
      case 9:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "Spybot.txt");
        mAutoMode = "Spy Bot";
        break;
      default:
        mAutoMode += " Pos: " + Integer.toString(mStartPosition);
        break;
    }
    System.out.println(mAutoMode);

    if (mGoForAutoLoader) {
      mAutoMode = "Go For Auto Loader";
    } else {
      mAutoMode = "No Auto Loading";
    }
    System.out.println(mAutoMode);
  }
}
