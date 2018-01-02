package missdaisy.autonomous;

import java.io.File;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import missdaisy.Constants;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.PathfinderController;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Shooter;

/**
 * Drive the specified segment of the trajectory
 * 
 */

public class DriveSegment extends State {
  private Drive mDrive;
  private Shooter mShooter;
  private Navigation mNavigation;
  private PathfinderController mPathfinder;
  private String mFile;
  private double startTime;
  private double delay;
  private boolean mDirection;
  private boolean useTurnPID;
  private boolean spinUpShooter;

  /**
   * @param distance The distance to travel, in say inches.
   * @param speed A number between -1.0 and 1.0
   */
  public DriveSegment(String filename, double direction, double useTurnPID, double spinShooter) {
    super("DriveSegement");
    System.out.println("Creating Drive Segment");
    mShooter = Shooter.getInstance();
    mDrive = Drive.getInstance();
    mPathfinder = PathfinderController.getInstance();
    mFile = "/home/lvuser/trajectories/" + filename + "/trajectory.csv";
    mDirection = direction < 0.0;
    this.useTurnPID = useTurnPID > 0.0;
    spinUpShooter = (spinShooter != 0.0);
    delay = spinShooter;
  }

  /**
   * Sets the drive base's current controller to be the drive distance controller
   */
  @Override
  public void enter() {
    System.out.println("Starting Pathfinder Segment " + mFile);

    // Reset the Pathfinder controller
    mPathfinder.reset();
    // mNavigation.resetEncoders();
    // mNavigation.resetRobotPosition(0.0, 0.0, 0.0);
    mPathfinder.useTurnCompensation(useTurnPID);

    File myFile = new File(mFile);
    Trajectory trajectory = Pathfinder.readFromCSV(myFile);
    mPathfinder.setTrajectory(trajectory, mDirection);

    mPathfinder.configureGains(0.5, 0.0, 0.0,
        1.0 / Constants.Properties.DRIVE_MAX_HIGH_GEAR_VELOCITY, 0.0);

    mDrive.setCurrentController(mPathfinder);
    System.out.println("Setting drive controller to pathfinder");
    
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void running() {
    /*
    if(spinUpShooter && (startTime + delay < Timer.getFPGATimestamp())) {
      mShooter.enableSpeedControlMode(2500);
    }
    */
  }

  /**
   * Ensures the robot's drive base is in an expected state.
   */
  @Override
  public void exit() {
    mDrive.setOpenLoop();
    mDrive.setSpeedTurn(0.0, 0.0);
    mPathfinder.useTurnCompensation(false);
  }

  /**
   * This state is considered done if the drive distance controller is on target
   */
  @Override
  public boolean isDone() {
    return mPathfinder.onTarget();
  }
}
