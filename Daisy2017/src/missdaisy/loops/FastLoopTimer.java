package missdaisy.loops;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.io.IOException;
import java.util.TimerTask;
import missdaisy.Constants;
import missdaisy.subsystems.DaisySubsystem;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Gearipor;
import missdaisy.subsystems.Hanger;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Lighting;
import missdaisy.subsystems.Shooter;
import missdaisy.subsystems.Turret;
import missdaisy.tracking.ReportList;
import missdaisy.tracking.RobotPose;
import missdaisy.tracking.TrackManager;
import missdaisy.utilities.Logger;

/**
 * @author jrussell
 */
public class FastLoopTimer extends TimerTask {

  private static FastLoopTimer instance = null;
  private java.util.Timer timer;
  private final DaisySubsystem[] subsystems =
      {Drive.getInstance(), Gearipor.getInstance(), Hanger.getInstance(), Intake.getInstance(),
          Lighting.getInstance(), Shooter.getInstance(), Turret.getInstance()};
  private final Navigation navigation = Navigation.getInstance();
  private final ReportList reportList = ReportList.getInstance();
  private final RobotPose poseList = RobotPose.getInstance();
  private PowerDistributionPanel panel;
  private Logger powerLogger;
  private double lastLogTime = -1;
  private double writeToFilePeriod = 180; // every 3 minutes
  private boolean log = false;

  public static FastLoopTimer getInstance() {
    if (instance == null) {
      instance = new FastLoopTimer();
    }
    return instance;
  }

  private FastLoopTimer() {
    File dir = new File("/home/lvuser/power_logs/");
    if (!dir.exists()) {
      dir.mkdirs();
    }
    String[] headers = {"Timestamp", "Voltage", "C0 Current", "C1 Current", "C2 Current",
        "C3 Current", "C4 Current", "C5 Current", "C6 Current", "C7 Current", "C8 Current",
        "C9 Current", "C10 Current", "C12 Current", "C13 Current", "C14 Current", "C15 Current"};
    String fileName = "PDP_Log_";
    Logger.Config loggerConfig = new Logger.Config(headers, headers.length, dir, fileName);
    
    powerLogger = new Logger(loggerConfig, 15);
    panel = new PowerDistributionPanel(Constants.CAN.PDP_ID);
    timer = new java.util.Timer();
  }

  public synchronized void start() {
    timer.scheduleAtFixedRate(this, 0L, Constants.Properties.FAST_LOOP_TIMER_PERIOD);
  }
  
  public synchronized void log(boolean use) {
    log = use;
  }
  
  public synchronized boolean isLogging() {
    return log;
  }
  
  public synchronized void writeLog() {
    try {
      powerLogger.write();
    } catch (IOException e) {
      e.printStackTrace();
    }
    powerLogger.clear();
  }

  @Override
  public void run() {    
    if (log) {
      Number[] row = new Number[18];
      row[0] = Timer.getFPGATimestamp();
      row[1] = panel.getVoltage();
      for (int i = 2; i < row.length; i++) {
        row[i] = panel.getCurrent(i - 2);
      }
      powerLogger.addRow(row);
    }
    
    if (lastLogTime == -1.0) {
      lastLogTime = Timer.getFPGATimestamp();
    }
    
    if (lastLogTime + writeToFilePeriod < Timer.getFPGATimestamp()) {
      writeLog();
      lastLogTime = Timer.getFPGATimestamp();
    }
    //double startTime = Timer.getFPGATimestamp();
    navigation.run();
    //poseList.run();
    //reportList.run();
    //TrackManager.getInstance().run();
    //SmartDashboard.putNumber("RunTimeOfTrackingStuff", Timer.getFPGATimestamp() - startTime);
    for (int i = 0; i < subsystems.length; i++) {
      subsystems[i].runInputFilters();
      subsystems[i].runCurrentController();
      subsystems[i].runOutputFilters();
    }
    
  }
}
