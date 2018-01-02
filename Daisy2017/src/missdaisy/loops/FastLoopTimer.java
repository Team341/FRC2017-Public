package missdaisy.loops;

import java.io.File;
import java.io.IOException;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import missdaisy.Constants;
import missdaisy.subsystems.DaisySubsystem;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Gearipor;
import missdaisy.subsystems.Hanger;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Lighting;
import missdaisy.subsystems.Shooter;
import missdaisy.utilities.Logger;

/**
 * @author jrussell
 */
public class FastLoopTimer extends TimerTask {

  private static FastLoopTimer instance = null;
  private java.util.Timer timer;
  private final DaisySubsystem[] subsystems =
      {Drive.getInstance(), Gearipor.getInstance(), Hanger.getInstance(), Intake.getInstance(),
          Lighting.getInstance(), Shooter.getInstance()};
  private final Navigation navigation = Navigation.getInstance();
  //private PowerDistributionPanel panel;
  private Logger powerLogger;
  private double lastLogTime = -1;
  private double writeToFilePeriod = 180; // every 3 minutes

  public static FastLoopTimer getInstance() {
    if (instance == null) {
      instance = new FastLoopTimer();
    }
    return instance;
  }

  private FastLoopTimer() {
    File dir = new File("/home/lvuser/power_logs");
    if (!dir.exists()) {
      dir.mkdirs();
    }
    String[] headers = {"Timestamp", "Voltage", "C0 Current", "C1 Current", "C2 Current",
        "C3 Current", "C4 Current", "C5 Current", "C6 Current", "C7 Current", "C8 Current",
        "C9 Current", "C10 Current", "C12 Current", "C13 Current", "C14 Current", "C15 Current"};
    String fileName = "PDP_Log_";
    Logger.Config loggerConfig = new Logger.Config(headers, headers.length, dir, fileName);
    powerLogger = new Logger(loggerConfig, 15);
    //panel = new PowerDistributionPanel(Constants.CAN.PDP_ID);
    timer = new java.util.Timer();
  }

  public synchronized void start() {
    timer.scheduleAtFixedRate(this, 0L, Constants.Properties.FAST_LOOP_TIMER_PERIOD);
  }

  @Override
  public void run() {    
    /*
    Number[] row = new Number[18];
    row[0] = Timer.getFPGATimestamp();
    row[1] = panel.getVoltage();
    for (int i = 2; i < row.length; i++) {
      row[i] = panel.getCurrent(i - 2);
    }
    powerLogger.addRow(row);
    
    
    if (lastLogTime == -1.0) {
      lastLogTime = Timer.getFPGATimestamp();
    }
    
    if (lastLogTime + writeToFilePeriod < Timer.getFPGATimestamp()) {
      try {
        powerLogger.write();
        powerLogger.clear();
      } catch (IOException e) {
        e.printStackTrace();
      }
      lastLogTime = Timer.getFPGATimestamp();
    }
    */
    navigation.run();
    
    for (int i = 0; i < subsystems.length; i++) {
      subsystems[i].runInputFilters();
      subsystems[i].runCurrentController();
      subsystems[i].runOutputFilters();
    }
    
  }
}
