package missdaisy.loops.controllers;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;
import missdaisy.Constants;
import missdaisy.loops.Navigation;
import missdaisy.loops.SynchronousPID;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.DaisyMath;
import missdaisy.utilities.Logger;

/**
 * PathfinderController.java This controller drives the robot along a specified trajectory using the
 * Pathfinder utility by Jaci found at https://github.com/JacisNonsense/Pathfinder
 *
 * @author AJN
 */

public class PathfinderController extends SynchronousPID implements Controller {

  private static PathfinderController pathfinderControllerInstance = null;

  public static Drive mDrive;
  public static Navigation mNavigation;

  public static TankModifier mModifier;

  // public static EncoderFollower left;
  // public static EncoderFollower right;

  public static Trajectory leftTraj;
  public static Trajectory rightTraj;

  private double kp = 0.0;
  private double ki = 0.0;
  private double kd = 0.0;
  private double kv = 1.0 / Constants.Properties.DRIVE_MAX_HIGH_GEAR_VELOCITY;
  private double ka = 0.0;
  private double last_left_error = 0.0;
  private double last_right_error = 0.0;
  private double current_heading = 0.0;
  private double initial_robot_heading = 0.0;
  private double initial_segment_heading = 0.0;
  private double initial_left_encoder_dist = 0.0;
  private double initial_right_encoder_dist = 0.0;
  private double lastVelocityLeft;
  private double lastVelocityRight;
  private double lastTime;
  private int current_segment;
  private boolean isBackwards = false;
  private boolean useTurnCompensation = false;
  private Logger logger;
  private double startTime = -1.0;


  public PathfinderController() {
    mDrive = Drive.getInstance();
    mNavigation = Navigation.getInstance();
    
    File dir = new File("/home/lvuser/pathfinder_logs");
    if (!dir.exists()) {
      dir.mkdirs();
    }
    
    String[] headers = {"Time", "Pathfinder Pos Left", "Measured Pos Left", "Pathfinder Vel Left",
                                "Measured Vel Left", "Pathfinder Acc Left", "Measured Acc Left", 
                                
                                "Pathfinder Pos Right", "Measured Pos Right", "Pathfinder Vel Right",
                                "Measured Vel Right", "Pathfinder Acc Right", "Measured Acc Right", 
                                "Pathfinder Heading", "Measured Heading"};
    String fileName = "PathFinderTrajectoryLog";
    Logger.Config loggerConfig = new Logger.Config(headers, headers.length, dir, fileName);

    logger = new Logger(loggerConfig, 10);
  }

  public static PathfinderController getInstance() {
    if (pathfinderControllerInstance == null) {
      pathfinderControllerInstance = new PathfinderController();
    }
    return pathfinderControllerInstance;
  }

  public synchronized void setTrajectory(Trajectory trajectory, boolean travelBackwards) {
    
    reset();
    
    // Convert the trajectory for use by a tank drive
    mModifier = new TankModifier(trajectory);
    mModifier.modify(Constants.Properties.DRIVE_WIDTH); // set the robot
    
    if (travelBackwards) {
      rightTraj = mModifier.getRightTrajectory();
      leftTraj = mModifier.getLeftTrajectory();
    } else {
      rightTraj = mModifier.getLeftTrajectory();
      leftTraj = mModifier.getRightTrajectory();
    }
    this.isBackwards = travelBackwards;
    initial_segment_heading = leftTraj.get(0).heading;
  }

  public synchronized void useTurnCompensation(boolean use) {
	this.useTurnCompensation = use;
  }
  
  public synchronized void configureDrive(int initialPositionLeft, int initialPositionRight) {

  }

  public synchronized void configureGains(double kp, double ki, double kd, double kv, double ka) {
    // Configure the PID/VA gains
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.kv = kv;
    this.ka = ka;
  }

  @Override
  public synchronized void run() {
    System.out.println("Running Pathfinder Controller");
    if (startTime == -1.0) {
      startTime = Timer.getFPGATimestamp();
    }
    if (onTarget()) {
      mDrive.setSpeed(0.0, 0.0);
    } else {

      double lSpeed = 0.0;
      double rSpeed = 0.0;
      double turn = 0.0;
      if (current_segment < leftTraj.length()) {
        Trajectory.Segment lSegment = leftTraj.get(current_segment);
        Trajectory.Segment rSegment = rightTraj.get(current_segment);

        double lError = 0.0;
        double rError = 0.0;
        double measuredPosL = (mNavigation.getLeftEncoderDistance() - initial_left_encoder_dist);
        double measuredPosR = (mNavigation.getRightEncoderDistance() - initial_right_encoder_dist);
        if (isBackwards){
          lError =
              lSegment.position * Constants.UnitConv.M_TO_IN + measuredPosL;
          rError =
              rSegment.position * Constants.UnitConv.M_TO_IN + measuredPosR;
        } else {
          lError =
              lSegment.position * Constants.UnitConv.M_TO_IN - measuredPosL;
          rError =
              rSegment.position * Constants.UnitConv.M_TO_IN - measuredPosR;
        }
        if (Constants.DEBUG_MODE) {
          SmartDashboard.putNumber("Pathfinder_lError", lError);
          SmartDashboard.putNumber("Pathfinder_rError", rError);
          
          SmartDashboard
              .putNumber("Pathfinder_lSegPos", lSegment.position * Constants.UnitConv.M_TO_IN);
          SmartDashboard
              .putNumber("Pathfinder_rSegPos", rSegment.position * Constants.UnitConv.M_TO_IN);
          SmartDashboard.putNumber("Pathfinder_lEncPos", mNavigation.getLeftEncoderDistance());
          SmartDashboard.putNumber("Pathfinder_rEncPos", mNavigation.getRightEncoderDistance());
        }

        double lVelocity = lSegment.velocity * Constants.UnitConv.MPS_TO_FPS;
        double rVelocity = rSegment.velocity * Constants.UnitConv.MPS_TO_FPS;

        lSpeed = DaisyMath
            .minmax(kp * lError + kd * ((lError - last_left_error) / lSegment.dt - lVelocity)
                + (kv * lVelocity + ka * lSegment.acceleration * Constants.UnitConv.MPS_TO_FPS), -1.0, 1.0);
        rSpeed = DaisyMath
            .minmax(kp * rError + kd * ((rError - last_right_error) / rSegment.dt - rVelocity)
                + (kv * rVelocity + ka * rSegment.acceleration * Constants.UnitConv.MPS_TO_FPS), -1.0, 1.0);

        double desired_heading = Math.toDegrees(DaisyMath.boundAngleNegPiToPiRadians(lSegment.heading - initial_segment_heading)); // Turn into right hand positive, pathfinder is reversed
        if (isBackwards) {
          lSpeed = -1.0*lSpeed;
          rSpeed = -1.0*rSpeed;
          //desired_heading = DaisyMath.boundAngleNeg180to180Degrees(heading + 180.0);
        }

        double gyro_heading = DaisyMath.boundAngleNeg180to180Degrees(mNavigation.getHeadingInDegrees() - initial_robot_heading); // Assuming the
        // value in degrees

        double angleDifference = DaisyMath
            .boundAngleNeg180to180Degrees(desired_heading - gyro_heading);
        
        double thisTime = Timer.getFPGATimestamp();
        double dt = thisTime - lastTime;
        double thisVelocityL = mNavigation.getLeftEncoderRate();
        double thisVelocityR = mNavigation.getRightEncoderRate();
        double leftAcc = (thisVelocityL - lastVelocityLeft) / dt;
        double rightAcc = (thisVelocityR - lastVelocityRight) / dt;
        lastVelocityLeft = thisVelocityL;
        lastVelocityRight = thisVelocityR;
        lastTime = thisTime;
        
        if (isBackwards) {
          measuredPosL *= -1.0;
          measuredPosR *= -1.0;
          thisVelocityL *= -1.0;
          thisVelocityR *= -1.0;
          leftAcc *= -1.0;
          rightAcc *= -1.0;
        }
        
        Number[] rowToAdd = new Number[] {Timer.getFPGATimestamp() - startTime, 
              lSegment.position * Constants.UnitConv.M_TO_IN, measuredPosL, 
              lVelocity, thisVelocityL, 
              lSegment.acceleration * Constants.UnitConv.MPS_TO_FPS, leftAcc, 
              
              rSegment.position * Constants.UnitConv.M_TO_IN, measuredPosR, 
              rVelocity, thisVelocityR,
              rSegment.acceleration * Constants.UnitConv.MPS_TO_FPS, rightAcc,
            
              desired_heading, gyro_heading, 
        };
      
        logger.addRow(rowToAdd);
        
        if (current_segment == leftTraj.length() - 1) {
          try {
            logger.write();
          } catch (IOException e) {
            e.printStackTrace();
          }
        }
        turn = DaisyMath.minmax(0.8 * (1.0 / 60.0) * angleDifference, -0.8, 0.8);
        
        //turn = 0; 
        

        current_segment++;

        if (Constants.DEBUG_MODE) {
          SmartDashboard.putNumber("Pathfinder_CurrentSegment", current_segment);
          SmartDashboard.putNumber("Pathfinder_lVelocity", lVelocity);
          SmartDashboard.putNumber("Pathfinder_rVelocity", rVelocity);
    
          SmartDashboard.putNumber("Pathfinder_lError", lError);
          SmartDashboard.putNumber("Pathfinder_rError", rError);
    
          SmartDashboard.putNumber("Pathfinder_lSpeed", lSpeed);
          SmartDashboard.putNumber("Pathfinder_rSpeed", rSpeed);
          SmartDashboard.putNumber("Pathfinder_lheading", Math.toDegrees(lSegment.heading));
          SmartDashboard.putNumber("Pathfinder_initial_segment_heading", Math.toDegrees(initial_segment_heading));
          SmartDashboard.putNumber("Pathfinder_initial_robot_heading", initial_robot_heading);
          SmartDashboard.putNumber("Pathfinder_pathAngleDelta", desired_heading);
          SmartDashboard.putNumber("Pathfinder_robotAngleDelta", gyro_heading);
          SmartDashboard.putNumber("Pathfinder_angDiff", angleDifference);
          SmartDashboard.putNumber("Pathfinder_turn", turn);
        }
      }
      
      mDrive.setSpeed(lSpeed + turn, rSpeed - turn);
    }
  }

  @Override
  public synchronized void reset() {
    mNavigation.resetEncoders();
    current_segment = 0;
    last_left_error = 0;
    last_right_error = 0;
    current_heading = mNavigation.getHeadingInDegrees();
    initial_left_encoder_dist = mNavigation.getLeftEncoderDistance();
    initial_right_encoder_dist = mNavigation.getRightEncoderDistance();
    initial_robot_heading = mNavigation.getHeadingInDegrees();
    initial_segment_heading = 0.0;
  }

  @Override
  public synchronized boolean onTarget() {
    // boolean isDone = left.isFinished() || right.isFinished();
    boolean isDone = current_segment >= leftTraj.length();
    if (isDone) {
      mDrive.setOpenLoop();
      mDrive.setSpeed(0.0, 0.0);
    }
    if (Constants.DEBUG_MODE){
      SmartDashboard.putBoolean("Pathfinder_onTarget", isDone);
    }
    return isDone;
  }

  @Override
  public synchronized void loadProperties() {
    // TODO Auto-generated method stub

  }

  @Override
  public String toString() {
    return "PathfinderController";
  }

}
