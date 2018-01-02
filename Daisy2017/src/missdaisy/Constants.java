package missdaisy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A list of all inputs, as well as other constants like RPM set points and PID
 * gains. These are just the port number that these components are plugged into
 * on the roborio.
 *
 * @authors Joshua Sizer, AJN
 */
public final class Constants {
  static {
    SmartDashboard.putBoolean("Debug Mode", false);
  }
  
  public static boolean DEBUG_MODE = false;
  
  public static void checkDebugMode() {
    DEBUG_MODE = SmartDashboard.getBoolean("Debug Mode", true);
  }

  public static class TrajectoryFiles {

    static final String TEST = "/home/lvuser/trajectories/test/myOtherFile.csv";
    static final String[] PATHS = {"/home/lvuser/trajectories/Blue/LeftGearToHopper/trajectory.csv",
        "/home/lvuser/trajectories/Blue/LeftSideGear/trajectory.csv",
        "/home/lvuser/trajectories/Blue/MiddleGear/trajectory.csv",
        "/home/lvuser/trajectories/Blue/MiddleGearToLeftHopper/trajectory.csv",
        "/home/lvuser/trajectories/Blue/MiddleGearToRightHopper/trajectory.csv",
        "/home/lvuser/trajectories/Blue/LeftGearToFarHopper/trajectory.csv",
        "/home/lvuser/trajectories/Blue/LeftGearToNearHopper/trajectory.csv",
        "/home/lvuser/trajectories/Blue/RightSideGear/trajectory.csv",
        "/home/lvuser/trajectories/Red/RightGearToFarHopper/trajectory.csv",
        "/home/lvuser/trajectories/Red/RightGearToNearHopper/trajectory.csv",
        "/home/lvuser/trajectories/Red/LeftSideGear/trajectory.csv",
        "/home/lvuser/trajectories/Red/MiddleGear/trajectory.csv",
        "/home/lvuser/trajectories/Red/MiddleGearToLeftHopper/trajectory.csv",
        "/home/lvuser/trajectories/Red/MiddleGearToRightHopper/trajectory.csv",
        "/home/lvuser/trajectories/Red/RightGearToHopper/trajectory.csv",
        "/home/lvuser/trajectories/Red/RightSideGear/trajectory.csv"};
    
    static final String[] RED_AUTONS = {
        "/home/lvuser/trajectories/DoNothing.txt",
        "/home/lvuser/trajectories/RedLeftAuto.txt",
        "/home/lvuser/trajectories/RedMiddleAuto.txt",
        "/home/lvuser/trajectories/RedRightAuto.txt",
        "/home/lvuser/trajectories/RedCloseHopperAuto.txt",
        "/home/lvuser/trajectories/RedFarHopperAuto.txt", //asdsadsa
        "/home/lvuser/trajectories/RedRightAutoFancy.txt",
        "/home/lvuser/trajectories/RedMiddleAutoShoot.txt",
        "/home/lvuser/trajectories/RedLeftToHumanAuto.txt"
        };
    
    static final String[] BLUE_AUTONS = {
        "/home/lvuser/trajectories/DoNothing.txt",
        "/home/lvuser/trajectories/BlueLeftAuto.txt",
        "/home/lvuser/trajectories/BlueMiddleAuto.txt",
        "/home/lvuser/trajectories/BlueRightAuto.txt",
        "/home/lvuser/trajectories/BlueCloseHopperAuto.txt",
        "/home/lvuser/trajectories/BlueFarHopperAuto.txt",
        "/home/lvuser/trajectories/BlueLeftAutoFancy.txt",
        "/home/lvuser/trajectories/BlueMiddleAutoShoot.txt",
        "/home/lvuser/trajectories/BlueRightToHumanAuto.txt"
        };
  }

  /**
   * All PWM inputs, mostly just for speed controllers.
   */
  public static class PWMs {

    public static final int DRIVE_LEFT_MOTOR = 0;
    public static final int DRIVE_RIGHT_MOTOR = 1;
    public static final int HANGER_MOTOR = 5;
    public static final int HANGER_MOTOR_2 = 6;

    public static final int INTAKE_MOTOR = 9;
    public static final int AGITATOR_MOTOR = 8;
    public static final int CONVEYOR_MOTOR = 7;
  }

  /**
   * Servo input numbers
   */
  public static final class SERVOs {
    
    public static final int GEARIPOR_UPPER_LEFT_SERVO = 17;  // 18
    public static final int GEARIPOR_UPPER_RIGHT_SERVO = 18; // 17
    
  }

  /**
   * Digital inputs, like encoders, banner sensors, or anything that has two
   * different outputs
   */
  public static final class DigitalInputs {

    public static final int DRIVE_LEFT_ENCODER_1 = 0;
    public static final int DRIVE_LEFT_ENCODER_2 = 1;
    public static final int DRIVE_RIGHT_ENCODER_1 = 2;
    public static final int DRIVE_RIGHT_ENCODER_2 = 3;

    public static final int GEARIPOR_HOOK_SENSOR = 8;
    //public static final int GEARIPOR_REVERSE_LIMIT = 8;
    //public static final int GEARIPOR_FORWARD_LIMIT = 9;
  }

  public static final class DigitalOutputs {
	  public static final int GEAR_ON_TARGET_SIGNAL = 5;
	  public static final int SHOOTER_ON_TARGET_SIGNAL = 6;
	  public static final int RUN_LIGHTSHOW_SIGNAL = 7;
  }
  
  
  /**
   * Solenoids are the components that control airflow in a pnuematic system.
   * Basically just a piston.
   */
  public static final class Solenoids {

    public static final int DRIVE_SHIFTER = 0;

    public static final int GEARIPOR_LOAD_GATE = 3;
    public static final int GEARIPOR_SCORE_GATE = 2;
    public static final int GEARIPOR_BALL_GATE = 1;
    
    
    public static final int ALLIANCE_RED_LED = 6;
    public static final int ALLIANCE_BLUE_LED = 7;

  }

  /**
   * Can is a special type of communication interface.
   */
  public static final class CAN {

    public static final int SHOOTER_TALONSRX_ID = 1;
    public static final int TURRET_TALONSRX_ID = 2;
    public static final int GEARIPOR_TALONSRX_ID = 3;
    public static final int PDP_ID = 20;
  }

  /**
   * Define the ports devices and sensors are located on
   */
  public static final class Cameras {
    public static final int TURRET_PIXY_ID = 0x54;
    public static final int GEARIPOR_PIXY_DEVICE_ID = 0x55;
    public static final double HEIGHT = 200;
    public static final double WIDTH = 320;
  }

  /*
   * These are the ports that the controllers are plugged into the driver
   * station, as well as the dead band for the controllers.
   */
  public static final class XBOXController {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double DEAD_BAND = 0.3;
  }

  public static final class Tracking {
    public static final double MAXIMUM_LOOKBACK_TIME = 0.75; //milliseconds (i.e. 2 seconds)
    public static final int POSE_HISTORY_LENGTH_CUTOFF = 50;
    public static final double LOOK_FORWARD_TIME = 10L; // This should be a few controller loop cycles
    public static final double CAMERA_FRAME_RATE = 50; // Pixy camera frame rate
    
    public static final int MAXIMUM_NUMBER_OF_TRACKS = 1;
    
    public static final double CORRELATION_RANGE = 100000.0; // inches, This is the distance threshold to meet to call a track and report similar 
  }

  // These define camera and target properties to convert pixel location into world coordinates
  public static final class Vision {
    public static final double BOILER_HEIGHT = 85.0; // inches, 78in to bottom of 2in ring, 84in to bottom of 4in strip
    public static final double BOILER_WIDTH = 15.0; // inches
    public static final double GEAR_HEIGHT =
        10.75 + 2.5; // inches,  10.75 in to bottom of target, 2.5in to halfway up target
    public static final double GEAR_SINGLE_WIDTH = 2.0; // inches, this is the width the tape on each side of the spring
    public static final double GEAR_TOTAL_WIDTH = 10.25; // inches
    public static final double TURRET_CAMERA_HEIGHT = 24.0; // inches
    public static final double TURRET_CAMERA_PITCH = 0.0; // deg
    public static final double HORIZONTAL_FOV = 75.0; // degrees
    public static final double VERTICAL_FOV = 43.6; // degrees
    public static final double SHOOTER_HORIZONTAL_OFFSET = 0.0; // degrees

    public static final double PIXEL_WIDTH = 320;
    public static final double PIXEL_HEIGHT = 200;
  }
  
  /**
   * Holds specific values for the physical relationships between robot subsystems
   */
  public static final class Physical {
	  public static final double GEARIPOR_X_FROM_ROBOT_CENTER = 0.0; 	// inches, distance forward of robot center
	  public static final double GEARIPOR_YAW_FROM_ROBOT_FORWARD = Math.PI; // rad, angle offset for the camera pointing direction compared to robot forward
	  
	  public static final double TURRET_X_FROM_ROBOT_CENTER = 1; 		// inches, distance forward of robot center
	  public static final double TURRET_Y_FROM_ROBOT_CENTER = 9; 	// inches, distance left/right of robot center
	  public static final double TURRET_YAW_OFFSET_ON_STARTUP = Math.toRadians(-90.0);
	  
	  public static final double CAMERA_X_FROM_TURRET_CENTER = 3.5; 	// inches, distance forward of turret center
	  public static final double CAMERA_Y_FROM_TURRET_CENTER = 4.75; 	// inches, distance left/right of turret center
	  public static final double CAMERA_YAW_FROM_TURRET = 0.0; 			// rad, angle offset from turret forward direction
	  public static final double CAMERA_HEIGHT = 21.0;
	  public static final double CAMERA_PITCH = 31.0;
  }

  /**
   * Holds specific values for properties, especially for PID
   */
  public static final class Properties {
    /**
     * Drive Parameters
     */
    // The gain for the drive's alpha filter, which is used to limit
    // acceleration
    public static final double DRIVE_ALPHA_FILTER_GAIN_LOW_SPEED = 0.1;
    public static final double DRIVE_ALPHA_FILTER_GAIN_HIGH_SPEED = 0.2;

    // The maximum percentage of power devoted towards turning (i.e. slower
    // turning)
    public static final double DRIVE_TURN_PERCENTAGE = 0.9;

    // Amount of power to provide to drive to prevent sliding
    public static final double DRIVE_BREAK_SPEED = 0.15;

    // Dual Speed gearbox parameters
    public static final double DRIVE_MAX_LOW_GEAR_VELOCITY = 5; // ft/s
    public static final double DRIVE_MAX_HIGH_GEAR_VELOCITY = 15; // ft/s
    public static final double DRIVE_HIGH_GEAR_SHIFT_SPEED_THRESHOLD = 4 * 12; // in/s 
    public static final double DRIVE_HIGH_GEAR_SHIFT_STICK_THRESHOLD = 0.5; // ft/s
    public static final double DRIVE_LEFT_LOW_GEAR_SERVO_ANGLE = 115; // deg
    public static final double DRIVE_LEFT_HIGH_GEAR_SERVO_ANGLE = 65; // deg
    public static final double DRIVE_RIGHT_LOW_GEAR_SERVO_ANGLE = 105; // deg
    public static final double DRIVE_RIGHT_HIGH_GEAR_SERVO_ANGLE = 65; // deg

    // The distance the robot moves per shaft rotation. Used to calculate
    // the speed of
    // the robot in the <code>Drive</code> class.
    public static final double DRIVE_DISTANCE_PER_PULSE = (Math.PI * 4) / 255;
    public static final double DRIVE_WHEEL_DIAMETER = 0.1016; // meters
    public static final int DRIVE_ENCODER_COUNTS_PER_REVOLUTION = 255;
    
    public static final double DRIVE_BACKUP_DISTANCE_TIME = 0.5;

    // Width between robot drive wheels
    public static final double DRIVE_WIDTH = 0.8382;  //meters

    /**
     * Drive Collision Parameters
     */
    public static final double DRIVE_COLLISION_DELTA_G_THRESHOLD = 0.5f;


    public static final double DEFAULT_SHOOTER_RPM = 3900;
    /**
     * Drive PID Parameters
     */
    // The acceptable deviation from the setpoint on the Drive Distance PID
    public static final double PID_DRIVE_DISTANCE_TOLERANCE = .5;

    // The acceptable deviation from the setpoint on the drive's turn PID
    public static final double PID_DRIVE_ANGLE_TOLERANCE = 1.0; // Normally
    // use 0.5

    // The maximum output to give to the DriveTurn PID, to aid in the
    // reduction of overshoot
    public static final double PID_DRIVE_TURN_MAX_OUTPUT = 0.5;

    // The minimum output to give to the DriveTurn PID, to aid in the
    // reduction of steady state
    // error
    public static final double PID_DRIVE_TURN_MIN_OUTPUT = 0.14;// 0.15;

    /**
     * Turret Parameters
     */
    public static final double SOFT_MAX_TURRET_ANGLE = -110.0;
    public static final double SOFT_MIN_TURRET_ANGLE = 210.0;
    public static final double TURRET_ROTATIONS_PER_TICK = 12.0 / 60;
    public static final double HARD_MAX_TURRET_ANGLE = 170.0;
    public static final double HARD_MIN_TURRET_ANGLE = -170.0;
    public static final double TURRET_ON_TARGET_TOLERANCE = 1.0; // deg
    
    
    public static final double[] RED_TURRET_ANGLES = new double[]{35.0, -75.0, 0.0};
    public static final double[] BLUE_TURRET_ANGLES = new double[]{0.0, 75.0, -35.0};
    

    /**
     * Shooter Parameters
     */
    // The PID/VA gains for the shooter speed control on the Talon SRX
    public static final double SHOOTER_KP = 0.22;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KV = 0.1097;    // This should be 1/(# rotations at max power)
    public static final double SHOOTER_KA = 0.0;

    // The speed the ball agitator should be run at
    public static final double BALL_AGITATOR_SPEED = 0.4;
    public static final double BALL_AGITATOR_UNJAM_SPEED = -0.25;

    // The speed the ball conveyor should be run at
    public static final double BALL_CONVEYOR_SPEED = 0.75;
    
    // This is the threshold for the shooter rpms to be considered on target
    public static final double SHOOTER_SPEED_TOLERANCE = 200; //rpm
    
    // This is the minimum RPMs the shooter must be at in order to feed fuel into it
    public static final double SHOOTER_MIN_SHOOTER_RPM = 1000; //rpm

    /**
     * Gearipor Servo Limits
     */
    public static final double GEARIPOR_RIGHT_LOADER_SERVO_OPEN = 70;
    public static final double GEARIPOR_RIGHT_LOADER_SERVO_CLOSED = 20;
    public static final double GEARIPOR_LEFT_LOADER_SERVO_OPEN = 60;
    public static final double GEARIPOR_LEFT_LOADER_SERVO_CLOSED = 110;
    
    
    public static final double GEARIPOR_RIGHT_SCORER_SERVO_OPEN = 0;
    public static final double GEARIPOR_RIGHT_SCORER_SERVO_CLOSED = 45;
    public static final double GEARIPOR_LEFT_SCORER_SERVO_OPEN = 45;
    public static final double GEARIPOR_LEFT_SCORER_SERVO_CLOSED = 0;
    
    /**
     * Gearipor general properties
     */
    public static final double GEARIPOR_LOAD_GATE_TIMEOUT = 1.0; // sec
    public static final double CONVEYOR_REVERSE_TIME = 0.5;
    
    /**
     * Intake Parameters
     */
    public static final double INTAKE_TURN_OFF_TIME_DELAY = 1000; // milliseconds (i.e. 0.5 sec)
    public static final double INTAKE_PASSIVE_SPEED = 0.25;
    
    
    /**
     * Controller Parameters
     */

    // The length, in milliseconds, of each loop for the fast loop timer
    // (which executes input and
    // output filters and the subsystem's current controllers
    public static final long FAST_LOOP_TIMER_PERIOD = 10L;
  }

  /**
   * Holds specific values for unit conversion
   */
  public static final class UnitConv {

    public static final double M_TO_IN = 39.3701;

    public static final double MPS_TO_FPS = 3.28084;

    public static final double RAD_TO_DEG = 180.0 / Math.PI;
    public static final double DEG_TO_RAD = Math.PI / 180.0;
  }
}
