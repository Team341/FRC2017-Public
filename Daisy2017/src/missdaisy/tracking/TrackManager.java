package missdaisy.tracking;

import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.utilities.DaisyMath;
import missdaisy.utilities.InterpolatingDouble;

public class TrackManager {
  private static TrackManager mTrackManager = null;
  Map<Integer, Track> TrackList = new TreeMap<>();
  private int nextTrackID = 0;
  private int bestBoilerTrack = -1;
  private int bestGearTrack = -1;

  private ReportList reportList = null;

  public static TrackManager getInstance() {
    if (mTrackManager == null) {
      mTrackManager = new TrackManager();
    }
    return mTrackManager;
  }

  public TrackManager() {
    reportList = ReportList.getInstance();
  }

  public Track getBestBoilerTrack() {
    if (bestBoilerTrack > 0) {
      return TrackList.get(bestBoilerTrack);
    } else {
      return null;
    }
  }

  public Track getBestGearTrack() {
    if (bestGearTrack > 0) {
      return TrackList.get(bestGearTrack);
    } else {
      return null;
    }
  }


  public void run() {
    // System.out.println("Running Report to Track Correlation...");
    double matchReportsStartTime = Timer.getFPGATimestamp();
    matchReportsToTrack();
    double matchReportsEndTime = Timer.getFPGATimestamp();

    // System.out.println("Updating Tracks...");
    double updateTracksStartTime = Timer.getFPGATimestamp();
    updateTracks();
    double updateTracksEndTime = Timer.getFPGATimestamp();

    // System.out.println("Prunning Tracks...");
    double updatePruneTracksStartTime = Timer.getFPGATimestamp();
    pruneTracks();
    double updatePruneTracksEndTime = Timer.getFPGATimestamp();

    // System.out.println("Determining Best Track...");
    double determineBestTracksStartTime = Timer.getFPGATimestamp();
    determineBestTracks();
    double determineBestTracksEndTime = Timer.getFPGATimestamp();

    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("Match reports to track dt",
          (matchReportsEndTime - matchReportsStartTime) * 1000);
      SmartDashboard.putNumber("Update tracks dt",
          (updateTracksEndTime - updateTracksStartTime) * 1000);
      SmartDashboard.putNumber("Update prune tracks dt",
          (updatePruneTracksEndTime - updatePruneTracksStartTime) * 1000);
      SmartDashboard.putNumber("Determine best tracks dt",
          (determineBestTracksEndTime - determineBestTracksStartTime) * 1000);
  
      SmartDashboard.putNumber("Number of Tracks", TrackList.size());
      SmartDashboard.putNumber("BestTrackID", bestBoilerTrack);
    }
    Track bestTrk = getBestBoilerTrack();
    if (bestTrk != null) {

      // Get the current robot pose
      RobotPose.Pose rPose =
          RobotPose.getInterpolated(new InterpolatingDouble(Timer.getFPGATimestamp()));;

      // Get the boilers range to the shooter (i.e. turret)
      ReferenceFrame turretPosInWF = rPose.turretFrame.mapTo(rPose.driveFrame);

      double rangeToBoiler =
          RobotPose.getInterpolated(new InterpolatingDouble(Timer.getFPGATimestamp())).turretFrame
              .findDistance(bestTrk.getPosition());
      double angleToBoilerInWF = turretPosInWF.findAngle(bestTrk.getPosition());
      double turretError =
          DaisyMath.boundAngleNegPiToPiRadians(angleToBoilerInWF - turretPosInWF.getYaw());

      if (Constants.DEBUG_MODE) {
      // Tell the turret to turn to the
        SmartDashboard.putNumber("Track X", bestTrk.getPosition().getX());
        SmartDashboard.putNumber("Track Y", bestTrk.getPosition().getY());
        SmartDashboard.putNumber("RangeToBoiler", rangeToBoiler);
        SmartDashboard.putNumber("AngleToBoiler", Math.toDegrees(angleToBoilerInWF));
      }

      // Convert the target into the robot frame
      ReferenceFrame targetInRobotFrame = rPose.driveFrame.rotateTo(bestTrk.getPosition());

      // Shift the target location relative to robot frame to the turret location
      ReferenceFrame turretFrame = new ReferenceFrame(Constants.Physical.TURRET_X_FROM_ROBOT_CENTER,
          Constants.Physical.TURRET_Y_FROM_ROBOT_CENTER, 0.0);
      ReferenceFrame targetInTurretFrame = turretFrame.rotateTo(targetInRobotFrame);

      double targetYawInTurretFrame =
          Math.toDegrees(Math.atan2(targetInTurretFrame.getY(), targetInTurretFrame.getX()));

      if (Constants.DEBUG_MODE) {
        SmartDashboard.putNumber("DesiredTurretYaw", targetYawInTurretFrame);
      }
    }
  }

  public void matchReportsToTrack() {

    Map<Double, Report> reports = reportList.getReportList();
    for (Iterator<Map.Entry<Double, Report>> rIterator = reports.entrySet().iterator(); rIterator
        .hasNext();) {
      // Compare each report to each track and see if it should be added to that track

      Map.Entry<Double, Report> rpt = rIterator.next();
      boolean matchFound = false;

      if (TrackList != null) {
        for (Iterator<Map.Entry<Integer, Track>> tIterator =
            TrackList.entrySet().iterator(); tIterator.hasNext();) {

          Map.Entry<Integer, Track> trk = tIterator.next();

          // Determine if report can be added to this track
          matchFound = trk.getValue().correlate(rpt.getKey(), rpt.getValue());

          if (matchFound) {
            // a match was made, stop searching for more matches
            // System.out.println("Match found for reportID " + rpt.getValue().ID);
            break;
          }
        }
      }

      if (!matchFound) {
        // No match was found, create a new track based on this report
        Track newTrk = Track.makeNewTrack(++nextTrackID, rpt.getKey(), rpt.getValue().signature,
            rpt.getValue().estimatedWorldPosition);

        // System.out.println("Creating new trackID " + newTrk.ID + " for reportID " +
        // rpt.getValue().ID);

        addTrack(newTrk);
      }

      // Remove the report from the report list
      reportList.getReportList().remove(rpt.getKey());
    }
  }


  public void addTrack(Track trk) {
    if (TrackList.size() < Constants.Tracking.MAXIMUM_NUMBER_OF_TRACKS) {
      TrackList.put(trk.ID, trk);
    }
  }

  public void updateTracks() {
    if (TrackList == null) {
      // System.out.println("Tracks list empty");
      return;
    }

    // Update the position estimate of each track
    for (Iterator<Map.Entry<Integer, Track>> iterator = TrackList.entrySet().iterator(); iterator
        .hasNext();) {
      Map.Entry<Integer, Track> entry = iterator.next();
      if (entry != null) {
        entry.getValue().update();
        /*
         * if (entry.getValue().isAlive()){ System.out.println("TrackID " + entry.getValue().ID +
         * " is alive"); } else { System.out.println("TrackID " + entry.getValue().ID + " is dead");
         * }
         */
      }

    }
  }

  public void pruneTracks() {
    if (TrackList == null) {
      return;
    }

    // Prune away dead tracks
    for (Iterator<Map.Entry<Integer, Track>> iterator = TrackList.entrySet().iterator(); iterator
        .hasNext();) {
      Map.Entry<Integer, Track> entry = iterator.next();
      if (!entry.getValue().isAlive()) {
        iterator.remove();
      }
    }
  }

  public void determineBestTracks() {
    // Loop over all the tracks and find the most stable track for both boiler and
    double boilerScore = 0;
    int boilerIndex = -1;
    double gearScore = 0;
    int gearIndex = -1;
    if (TrackList != null) {
      for (Iterator<Map.Entry<Integer, Track>> iterator = TrackList.entrySet().iterator(); iterator
          .hasNext();) {
        Map.Entry<Integer, Track> trk = iterator.next();
        double score = trk.getValue().getStability();
        if (trk.getValue().Signature == 1 && score > boilerScore) {
          boilerScore = score;
          boilerIndex = trk.getKey();
        }
        if (trk.getValue().Signature == 2 && score > gearScore) {
          gearScore = score;
          gearIndex = trk.getKey();
        }
      }
    }

    bestBoilerTrack = boilerIndex;
    bestGearTrack = gearIndex;
  }
}
