//package missdaisy.test;
//
//import java.awt.BorderLayout;
//import java.awt.Dimension;
//import java.text.DecimalFormat;
//import java.util.concurrent.TimeUnit;
//
//import javax.swing.JFrame;
//import javax.swing.JPanel;
//import javax.swing.SwingUtilities;
//
//import org.jfree.chart.ChartFactory;
//import org.jfree.chart.ChartPanel;
//import org.jfree.chart.JFreeChart;
//import org.jfree.chart.axis.NumberAxis;
//import org.jfree.chart.plot.PlotOrientation;
//import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
//import org.jfree.data.Range;
//import org.jfree.data.xy.XYSeries;
//import org.jfree.data.xy.XYSeriesCollection;
//
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
////import edu.wpi.first.wpilibj.Timer;
//import missdaisy.Constants;
//import missdaisy.Pixy;
//import missdaisy.loops.Navigation;
//import missdaisy.plotting.SquaredXYPlot;
//import missdaisy.tracking.ReferenceFrame;
//import missdaisy.tracking.Report;
//import missdaisy.tracking.ReportList;
//import missdaisy.tracking.RobotPose;
//import missdaisy.tracking.RobotPose.Pose;
//import missdaisy.tracking.Track;
//import missdaisy.tracking.TrackManager;
//import missdaisy.utilities.DaisyMath;
//
//public class TrackerTesting extends JFrame {
//
//  final static DecimalFormat fmt = new DecimalFormat("#0.000");
//  public static GaussianNoiseGenerator rangeNoiseGenerator = new GaussianNoiseGenerator(0, 8);
//  public static GaussianNoiseGenerator angleNoiseGenerator =
//      new GaussianNoiseGenerator(0, Math.toRadians(3));
//
//  public static class GaussianNoiseGenerator {
//
//    private double mean;
//    private double variance;
//    private java.util.Random r = new java.util.Random();
//
//    public GaussianNoiseGenerator(double mean, double variance) {
//      this.mean = mean;
//      this.variance = variance;
//    }
//
//    public double getNoise() {
//      return (r.nextDouble() - 0.5) * 2.0 * variance + mean;
//    }
//  }
//
//  // Simulation objections
//  private static double boilerDiameter = 12 + 9.5;
//  private static double boilerHeight = Constants.Vision.BOILER_HEIGHT;
//  private static ReferenceFrame boiler = new ReferenceFrame(0, 250, Math.toRadians(0));
//  private static ReferenceFrame robot = new ReferenceFrame(0, 75, Math.toRadians(0));
//  private static ReferenceFrame mTurret = new ReferenceFrame(0, -7.5, Math.toRadians(65));
//  private static ReferenceFrame camera = new ReferenceFrame(0, 5, Math.toRadians(0));
//  private static ReferenceFrame gear = new ReferenceFrame(-8.5, 0, Math.toRadians(0));
//
//  private static ReportList mReportList = ReportList.getInstance();
//  private static TrackManager mTrackManager = TrackManager.getInstance();
//
//  private static Track bTrk = null;
//  
//  public static void main(String[] args) {
//    // Open the plot
//    SwingUtilities.invokeLater(new Runnable() {
//      @Override
//      public void run() {
//        new TrackerTesting().setVisible(true);
//      }
//    });
//
//    Pause(1000);
//
//    double robotTurnRate = Math.toRadians(-36.0);
//    double robotSpeed = 6.0 * 12.0;
//    double turretYawRate = Math.toRadians(0.0);
//    double lastUpdateTime = 0.0;
//
//    double dt = 0.05;
//    for (double t = 0; t <= 10; t += dt) {
//      // Move the robot subsystems forward in time
//      robot = propagateFrame(robot, robotTurnRate, robotSpeed, dt);
//      mTurret = propagateFrame(mTurret, turretYawRate, 0, dt); // I.e. how much has the turret
//                                                               // turned this time step
//
//      // Figure out how the subsystems are relative to the new robot pos
//      ReferenceFrame tempTurret = mTurret.mapTo(robot);
//      ReferenceFrame tempCam = camera.mapTo(tempTurret);
//
//      // Figure out the relative information to create the detection report and relavant subsystem
//      // states
//      double rangeToBoiler = tempCam.FindDistance(boiler);
//      double angleToBoiler = tempCam.FindAngle(boiler);
//      double htaToBoiler = DaisyMath.boundAngleNegPiToPiRadians(angleToBoiler - tempCam.getYaw());
//
//      Pixy.Block camDetection = new Pixy.Block();
//      camDetection.range = rangeToBoiler + rangeNoiseGenerator.getNoise();
//      camDetection.yaw =
//          DaisyMath.boundAngleNegPiToPiRadians(htaToBoiler + angleNoiseGenerator.getNoise());
//      //camDetection.vAngle = Math.atan2(boilerHeight, rangeToBoiler);
//
//      RobotPose.Pose rPose = new Pose();
//      rPose.driveFrame = robot;
//      rPose.velocity = robotSpeed;
//      rPose.headingRate = robotTurnRate;
//      rPose.turretFrame = mTurret;
//      rPose.turretYawRate = turretYawRate;
//      rPose.cameraFrame = camera;
//      rPose.gearFrame = gear;
//
//      Report newRpt = new Report(t, 1, camDetection, rPose);
//
//      mReportList.Add(t, newRpt);
//
//      // Draw were we mapped the reports to
//      detections.add(newRpt.estimatedWorldPosition.getY(), newRpt.estimatedWorldPosition.getX());
//
//      //Timer.currTime = t;
//      if (bTrk == null){
//        bTrk = Track.makeNewTrack(1, t, 1, newRpt.estimatedWorldPosition);
//      }
//      
//      if ((t - lastUpdateTime) >= 0.25) {
//        bTrk.Correlate(t, newRpt);
//        
//        //mTrackManager.Run();
//        //lastUpdateTime = t;
//      }
//
//      if (bTrk != null) {
//        // Plot the position estimate of the best boiler track
//        bestTrk.add(bTrk.getPosition().getY(), bTrk.getPosition().getX());
//        
//        ReferenceFrame propagatedDriveFrame = rPose.driveFrame.propagateFrame(rPose.headingRate, rPose.velocity, 0.1);
//        
//        ReferenceFrame targetInRobotFrame = propagatedDriveFrame.rotateTo(bTrk.getPosition());
//        
//        // Shift the target location relative to robot frame to the turret location
//        ReferenceFrame turretFrame = new ReferenceFrame(Constants.Physical.TURRET_X_FROM_ROBOT_CENTER, Constants.Physical.TURRET_Y_FROM_ROBOT_CENTER, 0.0);
//        ReferenceFrame targetInTurretFrame = turretFrame.rotateTo(targetInRobotFrame);
//        
//        double targetYawInTurretFrame = Math.atan2(targetInTurretFrame.getY(), targetInTurretFrame.getX());
//        
//        
//        // Estimate what the correct turret angle should be and generate a turret yaw rate to always
//        // track the target
//        //double currentTurretYaw = DaisyMath.boundAngleNegPiToPiRadians(tempTurret.getYaw());
//        //double turretToBoiler_angle = tempTurret.FindAngle(bTrk.getPosition());
//
//        //ReferenceFrame propTurFrame = tempTurret.propagateFrame(rPose.headingRate, rPose.velocity, 0.1);
//        //double turretToBoiler_angle = propTurFrame.FindAngle(bTrk.getPosition());
//
//        double turretError = DaisyMath.boundAngleNegPiToPiRadians(targetYawInTurretFrame - mTurret.getYaw());
//        turretYawRate = 30 * turretError;
//
//        System.out.println("t=" + fmt.format(t)+ ", Robot Yaw: "
//            + fmt.format(Math.toDegrees(robot.getYaw())) + ", CurrentTurretAngle: "
//            + fmt.format(Math.toDegrees(mTurret.getYaw())) + ", DesiredTurretAngle:"
//            + fmt.format(Math.toDegrees(targetYawInTurretFrame)) + ", TurretError:"
//            + fmt.format(Math.toDegrees(turretError)) + ", TurretYawRate:"
//            + fmt.format(Math.toDegrees(turretYawRate)));
//        
//      }
//
//      /*
//       * // Print out some info to the console to help debug System.out.println("t=" + fmt.format(t)
//       * + ", Pos:[" + fmt.format(robot.getX()) + ", " + fmt.format(robot.getY()) +
//       * "], Range to Boiler:" + fmt.format(rangeToBoiler) + ", Angle to Boiler:" +
//       * fmt.format(Math.toDegrees(angleToBoiler)) + ", HTA: " +
//       * fmt.format(Math.toDegrees(htaToBoiler)));
//       */
//
//      // Update the graphic display
//      UpdatePlot();
//
//      // Pause for 50ms to allow user to see graphic updates
//      Pause(50);
//    }
//  }
//
//  public static ReferenceFrame propagateFrame(ReferenceFrame frame, double turn, double speed,
//      double dt) {
//    return new ReferenceFrame(frame.getX() + dt * speed * frame.getCosAng(),
//        frame.getY() + dt * speed * frame.getSinAng(), frame.getYaw() + dt * turn);
//  }
//
//  // Plot objects
//  private static ChartPanel trajPanel;
//  private static XYSeriesCollection plotObj = new XYSeriesCollection();
//  private static XYSeries robotFrame = new XYSeries("Robot", false, true);
//  private static XYSeries turretFrame = new XYSeries("Turret", false, true);
//  private static XYSeries cameraFrame = new XYSeries("Camera", false, true);
//  private static XYSeries gearFrame = new XYSeries("Gear", false, true);
//  private static XYSeries boilerFrame = new XYSeries("Boiler", false, true);
//  private static XYSeries robotHist = new XYSeries("Robot History", false, true);
//  private static XYSeries cameraHist = new XYSeries("Camera History", false, true);
//  private static XYSeries detections = new XYSeries("Detections", false, true);
//  private static XYSeries bestTrk = new XYSeries("BestTrkPosition", false, true);
//
//
//  public TrackerTesting() {
//    super("XY Line Chart Example with JFreechart");
//
//    // Draw the boiler
//    for (double n = 0; n < 2 * Math.PI; n += 2 * Math.PI / 20) {
//      boilerFrame.add(boilerDiameter * Math.sin(n) + boiler.getY(),
//          boilerDiameter * Math.cos(n) + boiler.getX());
//    }
//
//    plotObj.addSeries(robotFrame);
//    plotObj.addSeries(cameraFrame);
//    plotObj.addSeries(turretFrame);
//    plotObj.addSeries(gearFrame);
//    plotObj.addSeries(boilerFrame);
//    plotObj.addSeries(robotHist);
//    plotObj.addSeries(cameraHist);
//    // plotObj.addSeries(detections);
//    plotObj.addSeries(bestTrk);
//
//    // JFreeChart chart = ChartFactory.createXYLineChart("Tracking Result", "Down Field (in)",
//    // "Alliance Wall (in)", plotObj);
//
//    StandardXYItemRenderer r = new StandardXYItemRenderer();
//    // don't connect the dots
//    // r.setPlotLines(false);
//    // r.setPlotImages(true);
//    // show the points
//    r.setBaseShapesVisible(true);
//
//    // regular number axis
//    NumberAxis domainAx = new NumberAxis("Down Field (in)");
//    NumberAxis rangeAx = new NumberAxis("Alliance Wall (in)");
//    domainAx.setRange(new Range(-300.0, 300.0));
//    rangeAx.setRange(new Range(-300.0, 300.0));
//
//    SquaredXYPlot squarePlot = new SquaredXYPlot(plotObj, rangeAx, domainAx, r);
//    squarePlot.setOrientation(PlotOrientation.VERTICAL);
//
//    // define x-axis, and square y-axis to it
//    squarePlot.setSquaredToRange(false);
//
//    JFreeChart chart =
//        new JFreeChart("Tracking Result", JFreeChart.DEFAULT_TITLE_FONT, squarePlot, false);
//
//    trajPanel = new ChartPanel(chart);
//    trajPanel.setPreferredSize(new Dimension(640, 640));
//    add(trajPanel, BorderLayout.CENTER);
//
//    UpdatePlot();
//
//    setSize(640, 640);
//    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//    setLocationRelativeTo(null);
//  }
//
//  private static void UpdatePlot() {
//    // Draw the robot frame
//    robotFrame.clear();
//    double[] x = {10, 10, -10, -10, 10};
//    double[] y = {-15, 15, 15, -15, -15};
//    for (int n = 0; n < x.length; n++) {
//      robotFrame.add(x[n] * robot.getSinAng() + y[n] * robot.getCosAng() + robot.getY(),
//          x[n] * robot.getCosAng() - y[n] * robot.getSinAng() + robot.getX());
//    }
//
//    // Draw the turret
//    turretFrame.clear();
//    ReferenceFrame tempTurrFrame = mTurret.mapTo(robot);
//    turretFrame.add(30 * tempTurrFrame.getSinAng() + tempTurrFrame.getY(),
//        30 * tempTurrFrame.getCosAng() + tempTurrFrame.getX());
//    turretFrame.add(tempTurrFrame.getY(), tempTurrFrame.getX());
//    for (double n = 0; n < 2 * Math.PI; n += 2 * Math.PI / 20) {
//      turretFrame.add(4.5 * Math.sin(n) + tempTurrFrame.getY(),
//          4.5 * Math.cos(n) + tempTurrFrame.getX());
//    }
//
//
//    // Draw the camera
//    cameraFrame.clear();
//    double[] cx = {1, 1, -1, -1, 1};
//    double[] cy = {-1.5, 1.5, 1.5, -1.5, -1.5};
//    ReferenceFrame tempFrame = camera.mapTo(tempTurrFrame);
//    for (int n = 0; n < x.length; n++) {
//      cameraFrame.add(
//          cx[n] * tempFrame.getSinAng() + cy[n] * tempFrame.getCosAng() + tempFrame.getY(),
//          cx[n] * tempFrame.getCosAng() - cy[n] * tempFrame.getSinAng() + tempFrame.getX());
//    }
//
//    // Draw the gearipor
//    gearFrame.clear();
//    double[] gx = {2, 2, -2, -2, 2};
//    double[] gy = {-4.5, 4.5, 4.5, -4.5, -4.5};
//    ReferenceFrame tempGearFrame = gear.mapTo(robot);
//    for (int n = 0; n < x.length; n++) {
//      gearFrame.add(
//          gx[n] * tempGearFrame.getSinAng() + gy[n] * tempGearFrame.getCosAng()
//              + tempGearFrame.getY(),
//          gx[n] * tempGearFrame.getCosAng() - gy[n] * tempGearFrame.getSinAng()
//              + tempGearFrame.getX());
//    }
//
//    // Add the positions to the position histories
//    robotHist.add(robot.getY(), robot.getX());
//    cameraHist.add(tempFrame.getY(), tempFrame.getX());
//
//    trajPanel.repaint();
//  }
//
//  public static void Pause(int time) {
//    try {
//      TimeUnit.MILLISECONDS.sleep(time);
//    } catch (InterruptedException e) {
//      // TODO Auto-generated catch block
//      e.printStackTrace();
//    }
//
//  }
//}
