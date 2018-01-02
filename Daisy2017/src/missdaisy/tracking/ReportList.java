package missdaisy.tracking;

import java.util.Map;
import java.util.TreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.utilities.InterpolatingDouble;

/**
 * Keeps track of the various pixy packets and the state of the robot at the receive time. Used to
 * help interpolates & extrapolates target location.
 *
 * @author AJN
 */

public class ReportList {

  private static ReportList instance = new ReportList();
  
  private Map<Double, Report> reports = new TreeMap<>();
  private int reportID = 0;

  public static ReportList getInstance() {
    return instance;
  }

  private ReportList() {
    reports = new TreeMap<Double, Report>();
  }

  public void run() {
    // System.out.println("Creating Report");
    Vision.Block pkt = Vision.getInstance().getBlock();
    pkt.yaw = Math.toRadians(pkt.yaw);
    RobotPose.Pose pose = RobotPose.getInterpolated(new InterpolatingDouble(pkt.timeStamp));
    if (pose != null) {
      // System.out.println("TimeDelta between pkt and pose: " + (pkt.timeStamp - pose.timestamp));
      // System.out.println("Pkt yaw: " + Math.toDegrees(pkt.yaw));
      // System.out.println("Pkt Time: " + pkt.timeStamp);
      // System.out.println("PoseTime: " + pose.timestamp);
      // System.out.println("Current Time" + Timer.getFPGATimestamp());
      // System.out.println("Adding Report...");
      Report rpt = new Report(pkt.timeStamp, ++reportID, 1, pkt, pose);
      add(pkt.timeStamp, rpt);
    }
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("NumberOfReports", reports.size());
    }
  }

  public void add(double currTime, Report rpt) {
    // Put the new state into the history queue
    reports.put(currTime, rpt);
  }

  public Map<Double, Report> getReportList() {
    return reports;
  }
}
