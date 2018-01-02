package missdaisy.autonomous;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import missdaisy.Constants;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.PathfinderController;
import missdaisy.subsystems.Drive;

/**
 * Drive the specified segment of the trajectory
 * 
 */

public class DriveSegment extends State {
  private Drive mDrive;
  private Navigation mNavigation;
  private PathfinderController mPathfinder;
  private String mFile;
  private boolean mDirection;
  private boolean useTurnPID;

  /**
   * 
   * @param filename The relative path of the folder containing the trajectory's .csv and .txt file. 
   *    Example: "Red/MiddleGear" (without quotes)
   * @param direction Whether to run this trajectory in forward or reverse. 
   * @param useTurnPID Should we use our own PID on the heading error?
   */
  public DriveSegment(String filename, double direction, double useTurnPID) {
    super("DriveSegement");
    System.out.println("Creating Drive Segment");
    mDrive = Drive.getInstance();
    mPathfinder = PathfinderController.getInstance();
    mFile = "/home/lvuser/trajectories/" + filename + "/trajectory.csv";
    mDirection = direction < 0.0;
    this.useTurnPID = useTurnPID > 0.0;
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
  }

  @Override
  public void running() {}

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
