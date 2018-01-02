package missdaisy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Interface to laptop-based computer vision application.
 *
 * No vision processing is actually done here; rather, this is just a thin interface to
 * SmartDashboard to pull values that are set by an offboard program.
 *
 * @author Jared341
 */
public class Vision {

  private static Vision instance = new Vision();
  private static Block lastBlock;

  public static Vision getInstance() {
    return instance;
  }

  public static class Block {
    public double timeStamp;
    public double x;
    public double y;
    public double width;
    public double height;
    public double yaw;
    public double range;
  }

  private Vision() {
    lastBlock = new Block();
  }

  public boolean seesTarget() {
    return SmartDashboard.getBoolean("found", false);
  }

  public double getRPM() {
    return SmartDashboard.getNumber("rpms", 0.0);
  }

  public double getAzimuth() {
    return SmartDashboard.getNumber("azimuth", 0.0);
  }

  public double getRange() {
    return SmartDashboard.getNumber("range", 0.0);
  }

  public double getTime() {
    return SmartDashboard.getNumber("axis timestamp", 0.0);
  }

  public Block getBlock() {
    lastBlock.timeStamp = getTime();
    lastBlock.yaw = getAzimuth();
    lastBlock.range = getRange();

    return lastBlock;
  }
}
