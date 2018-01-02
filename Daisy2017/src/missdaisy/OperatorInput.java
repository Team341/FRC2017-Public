package missdaisy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import java.io.File;

import missdaisy.Constants.Properties;
import missdaisy.Constants.TrajectoryFiles;
import missdaisy.Constants.XBOXController;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.ShooterSpeedController;
import missdaisy.loops.controllers.AutoAimTurretController;
import missdaisy.loops.controllers.DriveTurnController;
import missdaisy.loops.controllers.PathfinderController;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Gearipor;
import missdaisy.subsystems.Hanger;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Lighting;
import missdaisy.subsystems.Shooter;
import missdaisy.subsystems.Turret;
import missdaisy.tracking.TrackManager;
import missdaisy.tracking.RobotPose;
import missdaisy.tracking.Track;
import missdaisy.utilities.DaisyMath;
import missdaisy.utilities.InterpolatingDouble;
import missdaisy.utilities.MovingAverageFilter;
import missdaisy.utilities.RateLimitFilter;
import missdaisy.utilities.XboxController;

/**
 * All the input from the operator and driver. This is the bread and butter of teleop mode
 *
 * @authors AJN, Josh Sizer
 */
public class OperatorInput {

  private static OperatorInput instance = null;
  protected XboxController mDriverController;
  protected XboxController mOperatorController;

  // subsystems
  private Drive mDrive;
  private Navigation mNavigation;
  private Shooter mShooter;
  private Intake mIntake;
  private Hanger mHanger;
  private Turret mTurret;
  private Gearipor mGearipor;
  private Lighting mLighting;
  private DigitalInput mSensor;

  // controllers
  private ShooterSpeedController mAutoAimShooter;
  private AutoAimTurretController mAutoAimTurret;
  private PathfinderController mPathfinder;

  // tracking
  private TrackManager mTrackManager;

  // operator input values
  private double mLeftMotorSpeed;
  private double mRightMotorSpeed;
  private double mSpeed;
  private double mTurn;
  private double mIntakeSpeed;
  private double mManualShooterSpeed;
  private double shooterRPM;
  private double mSpeedDecrease = 1.0;
  private long mShootStartTime = 0;
  private double mLastSpeedCmd = 0.0;

  private boolean mLastDriverAButtonState = false;
  private boolean mLastDriverBButtonState = false;
  private boolean mLastDriverXButtonState = false;
  private boolean mLastDriverYButtonState = false;
  private boolean mLastDriverLeftStickClick = false;
  private boolean mLastDriverRightStickClick = false;
  private boolean mLastDriverBackButtonState = false;
  private boolean mLastDriverStartButtonState = false;

  private boolean mLastOperatorAButtonState = false;
  private boolean mLastOperatorBButtonState = false;
  private boolean mLastOperatorXButtonState = false;
  private boolean mLastOperatorYButtonState = false;
  private boolean mLastOperatorLeftStickClick = false;
  private boolean mLastOperatorRightStickClick = false;
  private boolean mLastOperatorBackButtonState = false;
  private boolean mLastOperatorStartButtonState = false;

  // some booleans that control robot decisions
  private boolean mIsAutoAimControllers = false;
  private boolean mIsAutoAimOnTarget = false;
  private boolean mReadyToShoot = false;
  private boolean mResetHeading = true;
  private boolean mIsShooterOnTarget = false;
  private boolean mIsTurretOnTarget = false;

  private double loadGateTimerStart = 0.0;
  private double mStoppedShootingStartTime = 0.0;
  private double RPMSOffset = 0.0;
  private double RPMInitOffset = 300.0;

  private boolean mTriedToHang = false;
  private boolean mHangingStarted = false;
  private int mDeployingHangerCounter = 0;
  private double mLoadGateTimerStart = 0.0;
  private double mStartHangTime = 0.0;
  private boolean hanging = false;
  private MovingAverageFilter seesHookAverage;
  private RateLimitFilter driveOutputFilter;

  private boolean mArcadeDrive = true;

  private File myTrajectoryFile;
  private int mFileIndex = 0;

  public static OperatorInput getInstance() {
	if (instance == null) {
	  instance = new OperatorInput();
	}
    return instance;
  }

  private OperatorInput() {
    // Instantiate the gamepads
    mDriverController = new XboxController(Constants.XBOXController.DRIVER_PORT);
    mOperatorController = new XboxController(Constants.XBOXController.OPERATOR_PORT);
    driveOutputFilter = new RateLimitFilter(1.0/20.0);

    // Instantiate the subsystems
    mDrive = Drive.getInstance();
    mNavigation = Navigation.getInstance();
    mShooter = Shooter.getInstance();
    mIntake = Intake.getInstance();
    mHanger = Hanger.getInstance();
    mTurret = Turret.getInstance();
    mGearipor = Gearipor.getInstance();
    mLighting = Lighting.getInstance();
    
    mDrive.useAlphaFilter(false);

    // Instantiate the controllers
    mAutoAimShooter = ShooterSpeedController.getInstance();
    mAutoAimTurret = AutoAimTurretController.getInstance();
    mPathfinder = PathfinderController.getInstance();

    // Instantiate the track manager
    mTrackManager = TrackManager.getInstance();
    seesHookAverage = new MovingAverageFilter(20);

    // Populate Smartdashboard with some settable parameters
    SmartDashboard.putNumber("shooterRPM", 4600);
    SmartDashboard.putNumber("shooterVoltage", 1.0);
    SmartDashboard.putBoolean("Reverse Hanger", false);
    SmartDashboard.putBoolean("EnableShooterTesting", false);

    SmartDashboard.putNumber("Traj_kp", 0.25);
    SmartDashboard.putNumber("Traj_ki", 0.0);
    SmartDashboard.putNumber("Traj_kd", 0.0);
    SmartDashboard.putNumber("Traj_kv", 1.0 / Constants.Properties.DRIVE_MAX_HIGH_GEAR_VELOCITY);
    SmartDashboard.putNumber("Traj_ka", 0.0);
    SmartDashboard.putNumber("Turret Desired Angle Wrap Value", 0.0);

    SmartDashboard.putBoolean("EnableAutoShift", false);

    myTrajectoryFile = new File(Constants.TrajectoryFiles.PATHS[0]);
    SmartDashboard.putString("TrajectorFileName", myTrajectoryFile.getName());
    
    mSensor = new DigitalInput(Constants.DigitalInputs.GEARIPOR_HOOK_SENSOR);
    
  }

  public void processInputs() {
    /**
     * FOR TESTING: Cycle through the list of trajectory files
     */
    if (mDriverController.getStartButton() && !mLastDriverStartButtonState) {
      mFileIndex++;
      if (mFileIndex >= TrajectoryFiles.PATHS.length) {
        mFileIndex = 0;
      }
      myTrajectoryFile = new File(TrajectoryFiles.PATHS[mFileIndex]);
      SmartDashboard.putString("TrajectorFileName", myTrajectoryFile.getAbsolutePath());
    }
    
    /**
     * Run the tracker so we know where the goal objects are
     */
    // mTrackManager.Run();
    Track bestTrk = mTrackManager.getBestBoilerTrack();
    if (bestTrk != null) {
      double trkYawInWF = Math.atan2(bestTrk.getPosition().getY(), bestTrk.getPosition().getX());
      if (Constants.DEBUG_MODE) {
        SmartDashboard.putNumber("BestTrack_X", bestTrk.getPosition().getX());
        SmartDashboard.putNumber("BestTrack_Y", bestTrk.getPosition().getY());
        SmartDashboard.putNumber("BestTrack_Yaw", Math.toDegrees(trkYawInWF));
      }

      // Get the current robot pose
      RobotPose.Pose rPose =
          RobotPose.getInterpolated(new InterpolatingDouble(Timer.getFPGATimestamp()));
      if (rPose != null) {
        double trkYawInRobotFrame =
            DaisyMath.boundAngleNegPiToPiRadians(trkYawInWF - rPose.driveFrame.getYaw());
        if (Constants.DEBUG_MODE) {
          SmartDashboard.putNumber("BestTrack_YawInRobotFrame", Math.toDegrees(trkYawInRobotFrame));
        }
      }
    }
    // Checks if the auto aim is on target
    // as long as the current controllers are the auto aim ones,
    // and they are on target, set mReadyToShoot to true

    /*
     * mIsAutoAimControllers = mDrive.getCurrentController() == mAutoAimDrive &&
     * mShooter.getCurrentController() == mAutoAimShooter;
     * 
     * mIsAutoAimOnTarget = mAutoAimDrive.onTarget() && mAutoAimShooter.onTarget();
     * mIsShooterOnTarget = mShooterSpeed.onTarget() && mShooter.getCurrentController() ==
     * mShooterSpeed;
     * 
     * mReadyToShoot = (mIsAutoAimControllers && mIsAutoAimOnTarget) || mIsShooterOnTarget;
     * SmartDashboard.putBoolean("ReadyToShoot", mReadyToShoot);
     */
    mIsShooterOnTarget =
        mAutoAimShooter.onTarget() && mShooter.getCurrentController() == mAutoAimShooter;
    mIsTurretOnTarget =
        mAutoAimTurret.onTarget() && mTurret.getCurrentController() == mAutoAimTurret;

    //@formatter:off
    /**************************************************************************
     * Driver Inputs:
     *
     * Left Joystick - Arcade: Forward speed, Tank: Left Wheel Speed 
     * Right Joystick - Arcade: Turn speed/direction, Tank: Right Wheel Speed 
     * A Button - Run Shooter at RPM 
     * B Button - Follow Path Editor generated path going in forward direction 
     * X Button - Cancel currently running drive controllers 
     * Y Button - Follow Path Editor generated path going in reverse direction mode
     * Left Bumper
     * Right Bumper - 
     * Left Trigger - Shift to low gear 
     * Right Trigger - Shift to high gear 
     * Back Button - Reset Navigation setting (i.e. drive encoders) 
     * Start Button - toggle through list of trajectory files
     *
     **************************************************************************/
    //@formatter:on

    // Get the inputs for tank drive
    mLeftMotorSpeed =
        -1.0 * DaisyMath.applyDeadband(mDriverController.getLeftYAxis(), XBOXController.DEAD_BAND);

    mRightMotorSpeed =
        -1.0 * DaisyMath.applyDeadband(mDriverController.getRightYAxis(), XBOXController.DEAD_BAND);

    // Get the inputs for arcade drive
    mSpeed = mLeftMotorSpeed;
    
    if (Math.abs(mSpeed) < XBOXController.DEAD_BAND) {
      driveOutputFilter.reset();
    }
    
    // limit the rate of change of the output
    driveOutputFilter.setDesired(mSpeed);
    mSpeed = driveOutputFilter.run();
    
    mTurn = Properties.DRIVE_TURN_PERCENTAGE
        * DaisyMath.applyDeadband(mDriverController.getRightXAxis(), XBOXController.DEAD_BAND);

    // Determine if any drive controllers need to run
    if (mDriverController.getXButton()) {
      // Cancel any currently executing drive controllers
      mDrive.setOpenLoop();
      System.out.println("Canceling current controller");

      mShooter.disableSpeedControlMode();
      mShooter.setSpeed(0.0);
    } else if (mDrive.getCurrentController() == DriveTurnController.getInstance() && DriveTurnController.getInstance().onTarget()) {
      mDrive.setOpenLoop();
    } else if (mDrive.getCurrentController() == null) {

      // Check if the driver is specifying a controller to run
      if (mDriverController.getBButton() && !mLastDriverBButtonState) {
        // Follow a trajectory going forward
       // mDrive.setCurrentController(DriveTurnController.getInstance());
        myTrajectoryFile = new File("/home/lvuser/trajectories/Red/SmallBackup/trajectory.csv");
        Trajectory trajectory = Pathfinder.readFromCSV(myTrajectoryFile);
        
        mPathfinder.setTrajectory(trajectory, false);
        mPathfinder.configureGains(0.5, 0.0, 0.0,
            1.0 / Constants.Properties.DRIVE_MAX_HIGH_GEAR_VELOCITY, 0.0);
        mDrive.setCurrentController(PathfinderController.getInstance());
        /*

        System.out.println("Starting Pathfinder stuff");
        // Reset the Pathfinder controller
        // mPathfinder.reset();
        // mNavigation.resetRobotPosition(0.0, 0.0, 0.0);

        // Follow the trajectory generated by the path editor
        // File myFile = new File(Constants.TrajectoryFiles.PATHS[2]);
        Trajectory trajectory = Pathfinder.readFromCSV(myTrajectoryFile);

        // Create the Pathfinder controller and configure it
        System.out.println("	Setting trajectory to Pathfinder Controller...");
        mPathfinder.setTrajectory(trajectory, false);
        // mPathfinder.configureDrive(mNavigation.getLeftEncoderCounts(),
        // mNavigation.getRightEncoderCounts());

        // Load the Trajectory Following Controller parameters from the smart dashboard
        double kp = SmartDashboard.getNumber("Traj_kp", 0.25);
        double ki = SmartDashboard.getNumber("Traj_ki", 0.0);
        double kd = SmartDashboard.getNumber("Traj_kd", 0.0);
        double kv = SmartDashboard.getNumber("Traj_kv",
            1.0 / Constants.Properties.DRIVE_MAX_HIGH_GEAR_VELOCITY);
        double ka = SmartDashboard.getNumber("Traj_ka", 0.0);

        // Configure the PID/VA gains
        System.out.println("	Setting gains on trajectory followers...");
        mPathfinder.configureGains(kp, ki, kd, kv, ka);
        System.out.println("		kp = " + kp);
        System.out.println("		ki = " + ki);
        System.out.println("		kd = " + kd);
        System.out.println("		kv = " + kv);
        System.out.println("		ka = " + ka);

        // Set the pathfinder controller to be the active controller
        System.out.println("	Setting drive controller to Pathfinder controller");
        mDrive.setCurrentController(PathfinderController.getInstance());
        */

      } else if (mDriverController.getYButton() && !mLastDriverYButtonState) {
        // Follow a trajectory going backwards

        System.out.println("Starting Pathfinder stuff");
        // Reset the Pathfinder controller
        // mPathfinder.reset();
        // mNavigation.resetRobotPosition(0.0, 0.0, 0.0);

        // Follow the trajectory generated by the path editor
        // File myFile = new File(Constants.TrajectoryFiles.PATHS[2]);
        jaci.pathfinder.Trajectory trajectory = Pathfinder.readFromCSV(myTrajectoryFile);

        // Create the Pathfinder controller and configure it
        System.out.println("    Setting trajectory to Pathfinder Controller...");
        mPathfinder.setTrajectory(trajectory, true);
        // mPathfinder.configureDrive(mNavigation.getLeftEncoderCounts(),
        // mNavigation.getRightEncoderCounts());

        // Load the Trajectory Following Controller parameters from the smart dashboard
        double kp = SmartDashboard.getNumber("Traj_kp", 0.25);
        double ki = SmartDashboard.getNumber("Traj_ki", 0.0);
        double kd = SmartDashboard.getNumber("Traj_kd", 0.0);
        double kv = SmartDashboard.getNumber("Traj_kv",
            1.0 / Constants.Properties.DRIVE_MAX_HIGH_GEAR_VELOCITY);
        double ka = SmartDashboard.getNumber("Traj_ka", 0.0);

        // Configure the PID/VA gains
        System.out.println("    Setting gains on trajectory followers...");
        mPathfinder.configureGains(kp, ki, kd, kv, ka);
        System.out.println("        kp = " + kp);
        System.out.println("        ki = " + ki);
        System.out.println("        kd = " + kd);
        System.out.println("        kv = " + kv);
        System.out.println("        ka = " + ka);

        // Set the pathfinder controller to be the active controller
        System.out.println("    Setting drive controller to Pathfinder controller");
        mDrive.setCurrentController(PathfinderController.getInstance());

      } else if (mDriverController.getBackButton()) {
        // mNavigation.resetEncoders();
        mNavigation.resetRobotPosition(0, 0, 0);

      } else {
        // No Drive Controllers selected
        mDrive.setOpenLoop();
        DriveTurnController.getInstance().reset();
        DriveTurnController.getInstance().setGoal(DaisyMath.boundAngle0to360Degrees(Navigation.getInstance().getHeadingInDegrees() + 90.0));
      }

    } else {
      // Buttons in this branch of the if statement need to be held to remain active

    }

    // Apply the manual controls specified by the driver when no drive controller selected
    if (mDrive.getCurrentController() == null) {

      // Speed Shifting logic
      if (mDriverController.getLeftTrigger()) {
        // Down shift to low gear
        mDrive.setLowGear();
      } else if (mDriverController.getRightTrigger()) {
        // Shift up to high gear
        mDrive.setHighGear();
      } else {
        // Check if we should auto shift
        boolean allowAutoShift = SmartDashboard.getBoolean("EnableAutoShift", false);
        if (allowAutoShift) {
          // By default we should always be in high gear, however if we hit something going forward
          // or backwards, then shift to low gear to help push through it
          if (mNavigation.hasForwardCollision()) {
            mDrive.setLowGear();
          }

          /*
           * if (mDrive.autoShiftCheck() && Math.abs(mSpeed) >=
           * Properties.DRIVE_HIGH_GEAR_SHIFT_STICK_THRESHOLD) { // The robot is at the top end of
           * the low gear and the driver is still moving in that // direction, so shift to high gear
           * mDrive.setHighGear(); } else if (Math.abs(mSpeed) < 0.1 || (Math.signum(lastSpeedCmd)
           * != Math.signum(mSpeed))) { mDrive.setLowGear(); }
           */
        }
      }
      if (mDrive.isHighGear()){
        mDrive.setAlpha(Constants.Properties.DRIVE_ALPHA_FILTER_GAIN_HIGH_SPEED);
      } else {
        mDrive.setAlpha(Constants.Properties.DRIVE_ALPHA_FILTER_GAIN_LOW_SPEED);
      }
      
      mLastSpeedCmd = mSpeed;

      double intakeSpeed = -1.0 * DaisyMath.applyDeadband(mOperatorController.getRightYAxis(),
              Constants.XBOXController.DEAD_BAND);
      // Issue the drive commands
      if (mArcadeDrive) {
        mDrive.setSpeedTurn(mSpeed, mTurn);
        if (Math.abs(intakeSpeed) > 0.0) {
          mIntake.setIntakeSpeed(intakeSpeed);
        } else if (mSpeed > 0.0) {
          //mIntake.runIntake();	
          mIntake.setIntakeSpeed(Constants.Properties.INTAKE_PASSIVE_SPEED * (Math.abs(mNavigation.getAverageEncoderRate() / (Constants.Properties.DRIVE_MAX_HIGH_GEAR_VELOCITY * 12))));  
        } else {
          // should be 0 here
          mIntake.setIntakeSpeed(mIntakeSpeed);
        }
      } else {
        mDrive.setSpeed(mLeftMotorSpeed, mRightMotorSpeed);
        if (mLeftMotorSpeed > 0.0 && mRightMotorSpeed > 0.0) {
          mIntake.runIntake();
        } else {
          intakeSpeed = DaisyMath.applyDeadband(mOperatorController.getRightYAxis(),
              Constants.XBOXController.DEAD_BAND);
          mIntake.setIntakeSpeed(-0.5 * intakeSpeed);
          // mIntake.stopIntake();
        }
      }
    }

    // @formatter:off
    /**********************************************************************
     * Operator Inputs:
     * 
     * Left Joystick - Turn Turret
     * Left Joystick Click - Unjam ball feed
     * Right Joystick - Move Gearipor
     * A Button - Center Gearipor
     * B Button - Move Gearipor to the right
     * X Button - Move Gearipor to the left
     * Y Button - Run Ball Feed
     * Left Bumper - Hang
     * Right Bumper - Load Gear
     * Left Trigger - Turret auto aim/shoot
     * Right Trigger - Score Gear
     * Back Button - Reset Turret Angle
     * Start Button - Reset Gearipor Position
     * 
     **********************************************************************/
     // @formatter:on
    
    if (mOperatorController.getYButton()){
      mStoppedShootingStartTime = -1.0;
      mShooter.feedFuel();
      RPMInitOffset = Math.max(RPMInitOffset - 0.5, 0.0);
    } else {
      RPMInitOffset = 0.0;
      if (mStoppedShootingStartTime == -1) {
    	  mStoppedShootingStartTime = Timer.getFPGATimestamp();
          mShooter.setConveyorSpeed(-1.0 * Constants.Properties.BALL_CONVEYOR_SPEED);
      } else if (Timer.getFPGATimestamp() > mStoppedShootingStartTime + Properties.CONVEYOR_REVERSE_TIME) {
          mShooter.stopFeed();
      }
    }
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("RPMInitOffset", RPMInitOffset);
    }

    if (mOperatorController.getLeftStickClick() && !this.mLastOperatorLeftStickClick) {
      this.RPMSOffset += 50;
    } else if (mOperatorController.getRightStickClick() && !this.mLastOperatorRightStickClick) {
      this.RPMSOffset -= 50;
    }

    // Turret Controls
    if (mOperatorController.getBackButton()) {
      // For testing/debug, reset the turret angle back to 0.0
      mTurret.resetAngle();
    }
    
    /*
    DriverStation.Alliance color;
    color = DriverStation.getInstance().getAlliance();
    double[] turretAngles = new double[3];
    if(color == DriverStation.Alliance.Blue){
    	turretAngles = Constants.Properties.BLUE_TURRET_ANGLES;
    } else {
    	turretAngles = Constants.Properties.RED_TURRET_ANGLES;
    }
    */
    
    
    if (mOperatorController.getLeftTrigger()) {
      // Enable the turret auto-aim
      mTurret.setCurrentController(mAutoAimTurret);
      //mShooter.enableSpeedControlMode(SmartDashboard.getNumber("shooterRPM", 2000));
      mAutoAimShooter.setRPMSOffest(RPMSOffset + RPMInitOffset);
      mShooter.setCurrentController(mAutoAimShooter);
      if (mShooter.onTarget()) {
    	mLighting.setHoodLightOn(true);
      } else {
    	mLighting.setHoodLightOn(false);
      }
    } else if (mOperatorController.getXButton()) {
    	//mTurret.setAngle(turretAngles[0]);
      mTurret.setAngle(0.0);
    } else if (mOperatorController.getAButton()) {
    	//if (SmartDashboard.getBoolean("EnableShooterTesting", false)){
    		mShooter.enableSpeedControlMode(SmartDashboard.getNumber("shooterRPM", 2000) + RPMSOffset + RPMInitOffset);
    	//} else {
    	//	mTurret.setAngle(turretAngles[1]);
    	//}
    } else if (mOperatorController.getBButton()) {
    	//mTurret.setAngle(turretAngles[2]);
      mGearipor.openBallGate();
    } else {
      mGearipor.closeBallGate();
      mLighting.setHoodLightOn(false);
      // Manual control of the turret
      mTurret.setOpenLoop();
      mShooter.setOpenLoop();
      mShooter.disableSpeedControlMode();

      double turretSpeed = DaisyMath.applyDeadband(mOperatorController.getLeftXAxis(), .2);
      if (mDriverController.getAButton()) {
    	mTurret.setAngle(SmartDashboard.getNumber("Turret Desired Angle Wrap Value", 0.0));
      } else {
        //mTurret.setSpeed(-0.25 * turretSpeed);
        mTurret.setSpeed(-
            0.5 * turretSpeed);
      }
      mShooter.setSpeed(0.0);
    }
    SmartDashboard.putNumber("RPMS Offest", RPMSOffset);
    
    
    // Gearipor Controls
    
    /*
    if (mOperatorController.getRB()) {
      mGearipor.openLoader();
      //mGearipor.closeBallGate();
      mLoadGateTimerStart = Timer.getFPGATimestamp();
      // for the rainbow lights
      hanging = false;
    } else {
//      SmartDashboard.putNumber("GeariporTimout",
//          mLoadGateTimerStart + Constants.Properties.GEARIPOR_LOAD_GATE_TIMEOUT);
      if (Timer.getFPGATimestamp() > (mLoadGateTimerStart
          + Constants.Properties.GEARIPOR_LOAD_GATE_TIMEOUT)) {
        mGearipor.openLoader();
        if (!mOperatorController.getRightTrigger()) {
          //mGearipor.openBallGate();
        }
      } else {
        mGearipor.closeLoader();
        //mGearipor.closeBallGate();
      }
      // mGearipor.openBallGate();
    }
    */
    if (mOperatorController.getRB() && !mOperatorController.getBButton()){
      mGearipor.closeLoader();
    } else {
      mGearipor.openLoader();
    }
    
    /*
    if (mOperatorController.getRB()) {
    	mGearipor.openBallGate();
    } else {
    	mGearipor.closeBallGate();
    }
    
    */
    /*
    if (mOperatorController.getStartButton()) {
    	mGearipor.openLoader();
    } else {
    	mGearipor.closeLoader();
    }
    */
    
    
    double toAdd = (mSensor.get() ? 0.0 : 1.0);
    this.seesHookAverage.setInput(toAdd);
    seesHookAverage.run();
    
    //mLighting.setGearLightOn(seesHookAverage.getAverage() >= 0.8);
    double range = Vision.getInstance().getRange();
    
    if (Vision.getInstance().seesTarget() && range > 85.0 && range < 105.0) {
      mLighting.setGearLightOn(true);
    } else {
      mLighting.setGearLightOn(false);
    }
    
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putBoolean("SENSOR_OUTPUT", mSensor.get());
      SmartDashboard.putNumber("Hook Sensor Average Value", seesHookAverage.getAverage());
      }
    
   
    if (mOperatorController.getRightTrigger()) {
      mGearipor.closeBallGate();
      mGearipor.openScorer();
    } else {
      mGearipor.closeScorer();
    } 

    /*
     * if (mOperatorController.getStartButton()){ mGearipor.resetPositon(0.0); } else if
     * (mOperatorController.getAButton()) { mGearipor.setPosition(0.0); } else if
     * (mOperatorController.getBButton()) { mGearipor.setPosition(3.0); } else if
     * (mOperatorController.getXButton()) { mGearipor.setPosition(-3.0); } else {
     * mGearipor.setOpenLoop();
     * 
     * double gearSpeed = DaisyMath.applyDeadband(mOperatorController.getRightXAxis(),
     * Constants.XBOXController.DEAD_BAND); mGearipor.setSpeed(-0.40 * gearSpeed);
     * SmartDashboard.putNumber("GearMotorSpeed", gearSpeed); }
     * 
     * // Shooter if (mIsShooterOnTarget && mIsTurretOnTarget){ // Both the shooter and turret are
     * on target, start shooting mShooter.feedFuel(); } else if
     * (mOperatorController.getLeftStickClick()){ mShooter.unjamAgitator(); } else if
     * (mOperatorController.getYButton() && mShooter.getRPM() >
     * Constants.Properties.SHOOTER_MIN_SHOOTER_RPM){ mShooter.feedFuel(); } else {
     * mShooter.stopFeed(); }
     */


    boolean reverseHanger = SmartDashboard.getBoolean("Reverse Hanger", false);

    // Hanger
    if (mOperatorController.getLB()) {
      // Run the winch and start the light show
      
      mHanger.winch(reverseHanger);
      if (mStartHangTime == 0.0) {
    	mStartHangTime = Timer.getFPGATimestamp();
      }
      //mLighting.setRainbowMode(true);
    } else {
      // Stop running the winch
      mHanger.holdPosition();
      mStartHangTime = 0.0;
      hanging = false;
      //mLighting.setRainbowMode(hanging);
    }
    
    hanging = (mStartHangTime > 0.0 && mStartHangTime + 1.0 < Timer.getFPGATimestamp()) || hanging;
    /**
     * Keep track of what buttons the operators have pressed
     */
    // Driver game pad
    mLastDriverAButtonState = mDriverController.getAButton();
    mLastDriverBButtonState = mDriverController.getBButton();
    mLastDriverXButtonState = mDriverController.getXButton();
    mLastDriverYButtonState = mDriverController.getYButton();
    mLastDriverBackButtonState = mDriverController.getBackButton();
    mLastDriverStartButtonState = mDriverController.getStartButton();

    // Operator game pad
    mLastOperatorAButtonState = mOperatorController.getAButton();
    mLastOperatorBButtonState = mOperatorController.getBButton();
    mLastOperatorXButtonState = mOperatorController.getXButton();
    mLastOperatorYButtonState = mOperatorController.getYButton();
    mLastOperatorBackButtonState = mOperatorController.getBackButton();
    mLastOperatorStartButtonState = mOperatorController.getStartButton();
    mLastOperatorLeftStickClick = mOperatorController.getLeftStickClick();
    mLastOperatorRightStickClick = mOperatorController.getRightStickClick();

  }
}

