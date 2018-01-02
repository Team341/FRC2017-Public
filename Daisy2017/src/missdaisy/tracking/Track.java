package missdaisy.tracking;

import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import missdaisy.Constants;
import missdaisy.tracking.Report;

public class Track {
  int ID;
  int Signature;
  Map<Double, ReferenceFrame> positionHistory = new TreeMap<>();
  ReferenceFrame smoothedPosition;
  double width;
  double height;

  private Track() {

  }

  public static Track makeNewTrack(int id, double time, int signature,
      ReferenceFrame estimatedPos) {
    Track trk = new Track();

    // Set the ID number
    trk.ID = id;

    // Set the signature based on the detection signature
    trk.Signature = signature;

    // Add the report information into a queue
    // trk.reportHistory.put(time, detection);

    // Store the position history
    trk.positionHistory.put(time, new ReferenceFrame(estimatedPos));
    // System.out.println("TrackID " + id + " has history size: " + trk.positionHistory.size() + "
    // on creation");
    trk.smoothedPosition = new ReferenceFrame(estimatedPos);

    return trk;
  }

  public boolean isAlive() {
    return positionHistory.size() > 0;
  }

  public boolean correlate(double time, Report detection) {
    if (!isAlive()) {
      return false;
    }

    // Check if the estimated report position is close to this tracks
    // position
    double range = smoothedPosition.findDistance(detection.estimatedWorldPosition);
    if (this.Signature == detection.signature && range < Constants.Tracking.CORRELATION_RANGE) {
      positionHistory.put(time, detection.estimatedWorldPosition);
      // Update();
      return true;
    } else {
      // Update();
      return false;
    }
  }

  public void update() {

    // Prune old state blocks from the history queue
    double timeThreshold = Timer.getFPGATimestamp() - Constants.Tracking.MAXIMUM_LOOKBACK_TIME;
    for (Iterator<Map.Entry<Double, ReferenceFrame>> iterator =
        positionHistory.entrySet().iterator(); iterator.hasNext();) {
      Map.Entry<Double, ReferenceFrame> entry = iterator.next();
      if (entry.getKey() < timeThreshold) {
        iterator.remove();
      }
    }
    // System.out.println("TrkID " + this.ID + ", Num Points " + positionHistory.size());

    if (isAlive()) {
      // Average out the detection to get a smoothed estimate of the targets position
      double x = 0;
      double y = 0;
      for (Map.Entry<Double, ReferenceFrame> entry : positionHistory.entrySet()) {
        x += entry.getValue().positionX;
        y += entry.getValue().positionY;
      }
      x /= positionHistory.size();
      y /= positionHistory.size();
      smoothedPosition = new ReferenceFrame(x, y, 0.0);

    } else {
      smoothedPosition = null;
    }
  }

  public double getStability() {
    return Math.min(1.0, positionHistory.size()
        / (Constants.Tracking.CAMERA_FRAME_RATE * Constants.Tracking.MAXIMUM_LOOKBACK_TIME));
  }

  public ReferenceFrame getPosition() {
    return smoothedPosition;
  }
}
