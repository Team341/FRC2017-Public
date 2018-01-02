package missdaisy.tracking;

import java.util.Iterator;
import java.util.Map.Entry;
import edu.wpi.first.wpilibj.Timer;
import missdaisy.Constants;
import missdaisy.loops.Navigation;
import missdaisy.subsystems.Gearipor;
import missdaisy.subsystems.Turret;
import missdaisy.utilities.Interpolable;
import missdaisy.utilities.InterpolatingDouble;
import missdaisy.utilities.InterpolatingTreeMap;

public class RobotPose {
  public static RobotPose instance = new RobotPose();

  public static RobotPose getInstance() {
    return instance;
  }

  private static InterpolatingTreeMap<InterpolatingDouble, Pose> poses =
      new InterpolatingTreeMap<>();

  private RobotPose() {
    poses = new InterpolatingTreeMap<InterpolatingDouble, Pose>();
  }

  public static class Pose implements Interpolable<Pose> {
    public double timestamp;

    // Drive related items
    public ReferenceFrame driveFrame;
    public double velocity;
    public double headingRate;

    // Turret related items
    public ReferenceFrame turretFrame;
    public double turretYawRate;
    public ReferenceFrame cameraFrame;

    // Gearipor related items
    public ReferenceFrame gearFrame;
    public boolean topHatchOpen;
    public boolean bottomHatchOpen;
    public boolean seesHook;

    public Pose(Pose other) {
      if (other != null) {

        timestamp = other.timestamp;
        driveFrame = new ReferenceFrame(other.driveFrame);
        velocity = other.velocity;
        headingRate = other.headingRate;

        turretFrame = new ReferenceFrame(other.turretFrame);
        turretYawRate = other.turretYawRate;
        cameraFrame = new ReferenceFrame(other.cameraFrame);

        gearFrame = new ReferenceFrame(other.gearFrame);
        topHatchOpen = other.topHatchOpen;
        bottomHatchOpen = other.bottomHatchOpen;
        seesHook = other.seesHook;
      }
    }

    public Pose() {}

    @Override
    public Pose interpolate(Pose other, double x) {
      if (x <= 0) {
        return new Pose(this);
      } else if (x >= 1) {
        return new Pose(other);
      }
      return extrapolate(other, x);
    }

    public Pose extrapolate(Pose other, double x) {
      Pose interpPose = new Pose();
      interpPose.timestamp = x * (other.timestamp - timestamp) + timestamp;
      interpPose.driveFrame = driveFrame.interp(other.driveFrame, x);
      interpPose.velocity = x * (other.velocity - velocity) + velocity;
      interpPose.headingRate = x * (other.headingRate - headingRate) + headingRate;

      interpPose.turretFrame = turretFrame.interp(other.turretFrame, x);
      interpPose.turretYawRate = x * (other.turretYawRate - turretYawRate) + turretYawRate;
      interpPose.cameraFrame = cameraFrame.interp(other.cameraFrame, x);

      interpPose.gearFrame = gearFrame.interp(other.gearFrame, x);
      if (x > 0.5) {
        interpPose.topHatchOpen = other.topHatchOpen;
        interpPose.bottomHatchOpen = other.bottomHatchOpen;
        interpPose.seesHook = other.seesHook;
      } else {
        interpPose.topHatchOpen = topHatchOpen;
        interpPose.bottomHatchOpen = bottomHatchOpen;
        interpPose.seesHook = seesHook;
      }

      return interpPose;
    }
  }

  public void run() {
    // System.out.println("Adding Robot Pose...");
    // Create the pose of the robot at this time
    Pose rPose = new Pose();
    rPose.timestamp = Timer.getFPGATimestamp();
    rPose.driveFrame = Navigation.getInstance().getNavigationState().refFrame;
    rPose.velocity = Navigation.getInstance().getNavigationState().velocity;
    rPose.headingRate = Navigation.getInstance().getNavigationState().headingRate;

    rPose.turretFrame = new ReferenceFrame(Constants.Physical.TURRET_X_FROM_ROBOT_CENTER,
        Constants.Physical.TURRET_Y_FROM_ROBOT_CENTER,
        Math.toRadians(Turret.getInstance().getTurretAngle()));
    rPose.turretYawRate = Turret.getInstance().getTurretAngleRate();
    rPose.cameraFrame = new ReferenceFrame(Constants.Physical.CAMERA_X_FROM_TURRET_CENTER,
        Constants.Physical.CAMERA_Y_FROM_TURRET_CENTER, Constants.Physical.CAMERA_YAW_FROM_TURRET);

    rPose.gearFrame = new ReferenceFrame(Constants.Physical.GEARIPOR_X_FROM_ROBOT_CENTER,
        Gearipor.getInstance().getGearPosition(),
        Constants.Physical.GEARIPOR_YAW_FROM_ROBOT_FORWARD);
    rPose.topHatchOpen = Gearipor.getInstance().isLoaderOpen();
    rPose.bottomHatchOpen = Gearipor.getInstance().isScoringGateOpen();
    rPose.seesHook = Gearipor.getInstance().seesHook();

    // Put the new pose into the history queue
    poses.put(new InterpolatingDouble(rPose.timestamp), rPose);
    // System.out.println("CurrTime: " + rPose.timestamp + ", TurretYaw: " +
    // Math.toDegrees(rPose.turretFrame.getYaw()) + "ActualTurretYaw: " +
    // Turret.getInstance().getTurretAngle());

    prunePoses();
  }

  public static Pose getInterpolated(InterpolatingDouble key) {
    // TODO Auto-generated method stub
    return poses.getInterpolated(key);
  }

  public void prunePoses() {
    if (poses == null) {
      return;
    }

    // Prune away dead tracks
    for (Iterator<Entry<InterpolatingDouble, Pose>> iterator = poses.entrySet().iterator(); iterator
        .hasNext();) {
      Entry<InterpolatingDouble, Pose> entry = iterator.next();
      if ((Timer.getFPGATimestamp()
          - entry.getValue().timestamp) > Constants.Tracking.MAXIMUM_LOOKBACK_TIME) {
        iterator.remove();
      }
    }

    // System.out.println("Pose History Queue Length " + poses.size());
  }

}
