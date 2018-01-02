package missdaisy.test;

import java.util.Scanner;

import missdaisy.autonomous.StateMachine;
import missdaisy.fileio.PropertyReader;

public class AutonFileParserTest {
  private static final String kAutonomousFilePath = "C:\\Users\\ajned\\Documents\\Robotics\\Code\\FRC2017\\Daisy2017\\src\\Config\\";
  private static boolean mIsRedSide = true;
  private static int mNumAutoPos = 3;
  private static int mAutoPos = 2;
  private static boolean mGoForAutoLoader = false;
  private static String mAutoMode;
  private static String selector = "";
  private static Scanner in = new Scanner(System.in);
  private static PropertyReader mPropertyReader = new PropertyReader();
  private static StateMachine mStateMachine;

  public static void main(String[] args) {
    mPropertyReader = new PropertyReader();
    mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "BlueMiddleAuto.txt");
    
    
    //mStateMachine = new StateMachine(
    //    new AutonomousParser().parseStates(mIsRedSide, mAutoPos, mGoForAutoLoader));
  }
}
