package missdaisy;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;

import missdaisy.autonomous.AutonomousParser;
import missdaisy.autonomous.StateMachine;
import missdaisy.fileio.PropertyReader;
import missdaisy.loops.FastLoopTimer;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.DriveDistanceController;
import missdaisy.loops.controllers.DriveTurnController;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Gearipor;
import missdaisy.subsystems.Hanger;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Lighting;
import missdaisy.subsystems.Shooter;
import missdaisy.subsystems.Turret;

/**
 * The robot. This is the entry point for all functions and threads that are spawned. This class has
 * some key methods which include the autonomousPeriodic, teleopPeriodic, disabledPeriodic, and
 * testPeriodic. These are the functions that get called during that specific time during the match,
 * and are called every 20 ms.
 *
 * @authors AJN, Josh Sizer
 */

public class Daisy2017 extends IterativeRobot {

  // This reads in properties from some file location on the roborio.
  private PropertyReader mPropertyReader;

  // This provides the interface to log data variable to log files on the roborio
 // DataLogger dataLogger;

  // This controls the timing of the Controller threads.
  private FastLoopTimer mFastLoopTimer;

  // This is the operator's controller.
  private OperatorInput mOperatorInput;

  // This is the class which directs autonomous flow during
  // the autonomous period.
  private StateMachine mStateMachine;

  /*
   * These are just the paths on the roborio where we can find the properties files and autonomous
   * files. The reason we read some values in from a file is because it is quicker to connect over
   * FTP to the roborio's file system and change data inside a file, than it is to recompile code
   * with the new property value.
   */
  private static final String kPropertiesFilePath = "/home/lvuser/properties/properties.txt";
  private static final String kAutonomousFilePath = "/home/lvuser/autonomous/";

  /*
   * The "Last button state" booleans keep track of whether or not the button in question is being
   * held down. If the last button state is true, that means that the button on the controller has
   * not been let go since the last loop iteration.
   */
  boolean mLastDriverAButtonState = false;
  boolean mLastDriverBButtonState = false;
  boolean mLastDriverXButtonState = false;
  boolean mLastDriverYButtonState = false;
  boolean mLastOperatorAButtonState = false;
  boolean mLastOperatorBButtonState = false;
  boolean mLastOperatorXButtonState = false;
  boolean mLastOperatorYButtonState = false;

  /*
   * These are variables to allow for quick addition of different autonomous modes.
   */
  public static boolean mIsRedSide = true; // Which side of the field are we
  // starting on 0 is Red, 1 is Blue
  private int mStartPosition = 2; // 1 - close to hopper, 2 - middle of field,
  // 3 - far from hopper
  private int numStartPositions = 3;
  private boolean mGoForAutoLoader = false;
  String mAutoMode;
  private int fileIndex = 0;
  private String[] autonList;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /*
     * Here, we just instantiate or get the instance of the various objects
     */
    //CameraServer.getInstance().startAutomaticCapture();

    setAllianceColor();
    // Load configured properties
    mPropertyReader = new PropertyReader();
    mPropertyReader.parseFile(kPropertiesFilePath);
    loadAllProperties();

    // Instantiate our operator controls
    mOperatorInput = OperatorInput.getInstance();

    // starts the fast loop timer executing input & output filters and
    // controllers
    mFastLoopTimer = FastLoopTimer.getInstance();
    mFastLoopTimer.start();
    
    SmartDashboard.putBoolean("Is On Red Side", mIsRedSide);

    
    // Initialize a default autonomous mode. This will execute if the
    // drive/operator
    // does not choose to use another one.
    chooseAutoMode(mIsRedSide);
  }

  @Override
  public void disabledPeriodic() {
    if (mFastLoopTimer.isLogging()) {
      mFastLoopTimer.writeLog();
    }
    mFastLoopTimer.log(false);
	setAllianceColor();
	  
    /*
     * Just insure that nothing is trying to control the robot going into the start of autonomous
     */
    Drive.getInstance().setOpenLoop();
    Shooter.getInstance().setOpenLoop();
    Gearipor.getInstance().closeLoader();
    Turret.getInstance().setOpenLoop();
    Intake.getInstance().stopIntake();
    Hanger.getInstance().holdPosition();

    /**
     * This function is constantly called (remember, these functions are called in a loop) and
     * listens for a button press from the driver to change the autonomous mode before a match
     */
    listenForAutoChanges();
  }

  @Override
  public void autonomousInit() {
    setAllianceColor();

    /**
     * Instantiates the StateMachine, which ensure that the robot follows a very defined transition
     * between each state in autonomous. The AutonomousParser is a class that will read in an
     * autonomous file from the Roborio's file system, and parses it for the series of states that
     * the file contains.
     */
    mStateMachine = new StateMachine(
        new AutonomousParser().parseStates(mIsRedSide));
    logToDashboard();
    mFastLoopTimer.log(true);
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    /**
     * Runs the state machine
     */
    mStateMachine.run();
    logToDashboard();
  }

  @Override
  public void teleopInit() {
    mFastLoopTimer.log(true);
    setAllianceColor();
    Lighting.getInstance().endLightShow();
    //Lighting.getInstance().setRainbowMode(false);
    
    /**
     * Ensures that nothing is attempting to control the drive base at the very start of the
     * teleoperated period.
     */
    Drive.getInstance().setOpenLoop();
    logToDashboard();
  }

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    /*
     * This is the meat and potatoes of where decisions are made depending on what buttons are
     * pressed by the Operator and Driver.
     */
    mOperatorInput.processInputs();
    logToDashboard();
  }

  /**
   * This function is called periodically during test mode
   */
  @Override
  public void testPeriodic() {

  }
  
  public void setAllianceColor(){
    DriverStation.Alliance color;
    color = DriverStation.getInstance().getAlliance();
    if(color == DriverStation.Alliance.Blue){
      mIsRedSide = false;
      Lighting.getInstance().setAllianceLightBlue();
      autonList = Constants.TrajectoryFiles.BLUE_AUTONS;
    } else if (color == DriverStation.Alliance.Red){
      mIsRedSide = true;
      Lighting.getInstance().setAllianceLightRed();
      autonList = Constants.TrajectoryFiles.RED_AUTONS;
    }
  }

  /**
   * Listens for a button click from the Driver/Operator, indicating a change in autonomous modes.
   */
  private void listenForAutoChanges() {
    if (mOperatorInput.mDriverController.getAButton() && !mLastDriverAButtonState) {
      fileIndex++;
      if (autonList != null && fileIndex >= autonList.length){
        fileIndex = 0;
      }
      chooseAutoMode(mIsRedSide);
    }
    mLastDriverAButtonState = mOperatorInput.mDriverController.getAButton(); 
  }

  /**
   * Points the Property Reader to which autonomous file to parse, based on the inputs of:
   *
   * @param mIsRedSide Are we on the red side of the field or blue
   * @param mStartPosition Which position we start at (1, 2, 3)
   * @param mGoForAutoLoader Whether we want to auto load from the hopper near us
   */
  private void chooseAutoMode(boolean mIsRedSide) {
   
    if (autonList != null && fileIndex < autonList.length){
      mPropertyReader.parseAutonomousFile(autonList[fileIndex]);
      mAutoMode = autonList[fileIndex];
    } else {
      mPropertyReader.parseAutonomousFile("/home/lvuser/trajectories/DoNothing.txt");
      mAutoMode = "/home/lvuser/trajectories/DoNothing.txt";
    }
    
    SmartDashboard.putString("Autonomous Mode:", mAutoMode);
  }

  /**
   * Calls all of the Subsystems logToDashboard functions, which each publishes different values to
   * the SmartDashboard for debugging purposes.
   */
  public void logToDashboard() {
    // Check if we should publish debug values
    Constants.checkDebugMode();
    
    // Publish the desired outputs to the dashboard
    Navigation.getInstance().logToDashboard();
    Drive.getInstance().logToDashBoard();
    Gearipor.getInstance().logToDashboard();
    Hanger.getInstance().logToDashboard();
    Intake.getInstance().logToDashboard();
    Lighting.getInstance().logToDashboard();
    Shooter.getInstance().logToDashboard();
    Turret.getInstance().logToDashboard();    
  }

  /**
   * This tells each subsystem to load subsystem specific properties.
   */
  public void loadAllProperties() {
    DriveTurnController.getInstance().loadProperties();
    DriveDistanceController.getInstance().loadProperties();
  }
}
