package missdaisy;

import java.util.TreeMap;

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
  
  private TreeMap<Double, Double> rangeTable;
  private double kRangeOffset = 0.0;
  
  private Vision() {
    rangeTable = new TreeMap<Double, Double>();
    rangeTable.put(80.0, 3700.0);
    //rangeTable.put(85.0, 3180.0);
    rangeTable.put(90.0, 3750.0);
    rangeTable.put(95.0, 3850.0);
    rangeTable.put(100.0, 4000.0);
  }
  
  public double getRPMsForRange(double range) {
    double lowKey = -1.0;
    double lowVal = -1.0;
    for (double key : rangeTable.keySet()) {
      if (range < key) {
        double highVal = rangeTable.get(key);
        if (lowKey > 0.0) {
          double m = (range - lowKey) / (key - lowKey);
          return lowVal + m * (highVal - lowVal);
        } else
          return highVal;
      }
      lowKey = key;
      lowVal = rangeTable.get(key);
    }

    return 3750 + kRangeOffset;
  }

  public static Vision getInstance() {
    return instance;
  }
  
  public boolean seesTarget() {
    return SmartDashboard.getBoolean("found", false);
  }

  public double getRPM() {
    return getRPMsForRange(getRange());
  }

  public double getAzimuth() {
    /*
    double camAzimuthDeg = SmartDashboard.getNumber("azimuth", 0.0);
    double camAzimuthRad = Math.toRadians((90 - camAzimuthDeg));
    double dist = getRange();
    double cam_x = dist * Math.cos(camAzimuthRad);
    double cam_y = dist * Math.cos(camAzimuthRad);
    
    double target_x = cam_x + Constants.Physical.CAMERA_X_FROM_SHOOTER_CENTER;
    double target_y = cam_y + Constants.Physical.CAMERA_Y_FROM_SHOOTER_CENTER;
    
    double azimuth = Math.toDegrees(Math.atan2(target_y, target_x));
    */
    return SmartDashboard.getNumber("azimuth", 0.0);
  }

  public double getRange() {
    return SmartDashboard.getNumber("range", 0.0);
  }
}
