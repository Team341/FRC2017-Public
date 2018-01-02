package missdaisy.tracking;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.tracking.RobotPose.Pose;

public class Report {

  public double receivedTime;
  public int signature;
  public Vision.Block pkt;
  public Pose robotPose;
  public ReferenceFrame estimatedWorldPosition;
  public boolean validReport = true;
  public int ID;

  public Report(double time, int id, int signature, Vision.Block pkt, Pose robotPose) {
    this.ID = id;
    this.receivedTime = time;
    this.signature = signature;
    this.pkt = pkt;
    this.robotPose = robotPose;
    this.estimatedWorldPosition = estimateWorldPosition();
  }

  public ReferenceFrame estimateWorldPosition() {
    ReferenceFrame targetPosInWorldFrame = null;

    // Convert the pixy measurements into an xy position relative to
    // the camera
    ReferenceFrame targetPosInCameraFrame =
        new ReferenceFrame(pkt.range * Math.cos(pkt.yaw), pkt.range * Math.sin(pkt.yaw), 0.0);
    // System.out.println("reportID: " + this.ID);
    // System.out.println(" TargetPosInCameraFrame: [" + targetPosInCameraFrame.positionX + ", " +
    // targetPosInCameraFrame.positionY + "]");
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("TargPosInCameraFrameX", targetPosInCameraFrame.positionX);
      SmartDashboard.putNumber("TargPosInCameraFrameY", targetPosInCameraFrame.positionY);
      SmartDashboard.putNumber("TargPosInCameraFrameYaw",
          Math.toDegrees(targetPosInCameraFrame.getYaw()));
    }

    // Rotate target point from camera frame to turret frame
    ReferenceFrame targetPosInTurretFrame =
        robotPose.cameraFrame.rotateFrom(targetPosInCameraFrame);
    // System.out.println(" TargetPosInTurretFrame: [" + targetPosInTurretFrame.positionX + ", " +
    // targetPosInTurretFrame.positionY + "], CameraYaw: " +
    // Math.toDegrees(robotPose.cameraFrame.getYaw()) + ", Target Yaw: " +
    // Math.toDegrees(targetPosInCameraFrame.getYaw()));
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("TargPosInTurretFrameX", targetPosInTurretFrame.positionX);
      SmartDashboard.putNumber("TargPosInTurretFrameY", targetPosInTurretFrame.positionY);
      SmartDashboard.putNumber("TargPosInTurretFrameYaw",
          Math.toDegrees(targetPosInTurretFrame.getYaw()));
    }
    // Rotate target point from turret frame to robot frame
    ReferenceFrame targetPosInRobotFrame = robotPose.turretFrame.rotateFrom(targetPosInTurretFrame);
    // System.out.println(" TargetPosInRobotFrame: [" + targetPosInRobotFrame.positionX + ", " +
    // targetPosInRobotFrame.positionY + "], TurretYaw: " +
    // Math.toDegrees(robotPose.turretFrame.getYaw()) + ", Target Yaw: " +
    // Math.toDegrees(targetPosInTurretFrame.getYaw()));
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("TargPosInRobotFrameX", targetPosInRobotFrame.positionX);
      SmartDashboard.putNumber("TargPosInRobotFrameY", targetPosInRobotFrame.positionY);
      SmartDashboard.putNumber("TargPosInRobotFrameYaw",
          Math.toDegrees(targetPosInRobotFrame.getYaw()));
    }
    // Rotate target point from robot frame to world frame
    targetPosInWorldFrame = robotPose.driveFrame.rotateFrom(targetPosInRobotFrame);
    // System.out.println(" TargetPosInWorldFrame: [" + targetPosInWorldFrame.positionX + ", " +
    // targetPosInWorldFrame.positionY + "], RobotYaw: " +
    // Math.toDegrees(robotPose.driveFrame.getYaw()) + ", Target Yaw: " +
    // Math.toDegrees(targetPosInRobotFrame.getYaw()));
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("TargPosInWorldFrameX", targetPosInWorldFrame.positionX);
      SmartDashboard.putNumber("TargPosInWorldFrameY", targetPosInWorldFrame.positionY);
      SmartDashboard.putNumber("TargPosInWorldFrameYaw",
          Math.toDegrees(targetPosInWorldFrame.getYaw()));
    }

    /*
     * if (i > 0){ // Check that the centers of the detections are within the actual width of the
     * boiler if (tempPos[i].FindDistance(tempPos[i-1]) > Constants.Vision.BOILER_WIDTH){ // These
     * detections are not likely to be of the boiler and should be rejected validReport = false; } }
     */

    return targetPosInWorldFrame;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Report) {
      Report rpt = (Report) obj;
      return this.receivedTime == rpt.receivedTime;
    }
    return false;
  }
}
