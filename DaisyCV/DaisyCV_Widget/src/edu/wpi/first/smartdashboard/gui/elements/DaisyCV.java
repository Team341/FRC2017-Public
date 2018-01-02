package edu.wpi.first.smartdashboard.gui.elements;

import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.smartdashboard.properties.DoubleProperty;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Scanner;
import java.util.TreeMap;

import javax.imageio.ImageIO;

import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_core.CvMemStorage;
import org.bytedeco.javacpp.opencv_core.CvRect;
import org.bytedeco.javacpp.opencv_core.CvSeq;
import org.bytedeco.javacpp.opencv_core.CvSize;
import org.bytedeco.javacpp.opencv_core.IplConvKernel;
import org.bytedeco.javacpp.opencv_core.IplImage;
import org.bytedeco.javacpp.opencv_imgproc;
import org.bytedeco.javacv.CanvasFrame;

import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.wpijavacv.*;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;

/*
 * HOW TO GET THIS COMPILING IN NETBEANS:
 * 
 * 1. Install the SmartDashboard using the installer (if on Windows) 1a. Verify that the OpenCV
 * libraries are in your PATH (on Windows) 2. Add the following libraries to the project:
 * SmartDashboard.jar extensions/WPICameraExtension.jar lib/NetworkTable_Client.jar
 * extensions/lib/javacpp.jar extensions/lib/javacv-*your environment*.jar extensions/lib/javacv.jar
 * extensions/lib/WPIJavaCV.jar
 *
 */
/**
 *
 * @author jrussell
 */
public class DaisyCV extends WPICameraExtension {
  private static final long serialVersionUID = 1870790236426202916L;
  public static final String NAME = "DaisyCV 2017";
  private WPIColor targetColor = new WPIColor(0, 255, 0);
  private NetworkTable SmartDashboard;
  // private PropertySet properties;

  // Constants that need to be tuned
  // basically, the horizontal slope is the acceptable tilt of the lines found
  private static final double kNearlyHorizontalSlope = Math.tan(Math.toRadians(20));
  private static final double kNearlyVerticalSlope = Math.tan(Math.toRadians(90 - 30));
  private static final int kMinWidth = 20;
  private static final int kMaxWidth = 150;
  private static final double kRangeRPMOffset = 0.0;
  private static final double kRangeOffset = 10.0;
  private static final int kHoleClosingIterations = 3;

  // private static double pShooterOffsetDeg = 1.0;
  private static final double kHorizontalFOVDeg = 47.0;
  private static final double kVerticalFOVDeg = 480.0 / 640.0 * kHorizontalFOVDeg;
  // focal length = (image width) / (2 * tan(FOV/2))
  // FOV = 2 * atan(width/ (2 * Focal length))
  private static final double kFocalLength = 640 / 2 / Math.tan(Math.toRadians(47.0) / 2);

  private static final double kCameraHeightIn = 22.0;
  private static final double kCameraPitchDeg = 35.0;
  private static final double kTargetHeightIn = 86.0;


  // 82 to bottom of target, 13 to middle to bottom of target
  // p for properties, so that editing the program can be done on the fly
  // they are initialized to default values, and these values are used if
  // a properties file cannot be found

  private IntegerProperty pHueLowerBound = new IntegerProperty(this, "Hue low", 45);
  private IntegerProperty pHueUpperBound = new IntegerProperty(this, "Hue high", 75);
  private IntegerProperty pSaturationLowerBound = new IntegerProperty(this, "Saturation low", 154);
  private IntegerProperty pSaturationUpperBound = new IntegerProperty(this, "Saturation high", 255);
  private IntegerProperty pValueLowerBound = new IntegerProperty(this, "Value low", 135);
  private IntegerProperty pValueUpperBound = new IntegerProperty(this, "Value high", 255);
  private DoubleProperty pShooterOffsetDeg =
      new DoubleProperty(this, "Horizontal Offset Degrees", 0);
  private BooleanProperty downloadImages = new BooleanProperty(this, "Download images");
  private double downloadPeriod = 4000;
  private double lastDownloadTime = 0.0;
  private int pictureCounter = 0;

  private TreeMap<Double, Double> rangeTable;

  private boolean m_debugMode = true;

  // Store JavaCV temporaries as members to reduce memory management during
  // processing
  private CvSize size = null;
  private WPIContour[] contours;
  private ArrayList<Target> targets;
  private IplConvKernel morphKernel;
  private IplImage bin;
  private IplImage hsv;
  private IplImage hue;
  private IplImage sat;
  private IplImage val;
  private WPIPoint linePt1;
  private WPIPoint linePt2;
  private int horizontalOffsetPixels;
  private double timestamp;

  public DaisyCV() {
    this(false);
  }

  private class Target {
    public double centroidX;
    public double centroidY;
    public double width;
    public double height;
    public WPIContour countours;
  }

  public DaisyCV(boolean debug) {
    m_debugMode = debug;
    morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);

    rangeTable = new TreeMap<Double, Double>();
    rangeTable.put(80.0, 3091.0);
    // rangeTable.put(85.0, 3180.0);
    rangeTable.put(90.0, 3180.0);
    rangeTable.put(95.0, 3261.0);
    rangeTable.put(100.0, 3350.0);
    // rangeTable.put(105.0, 3000.0);
    rangeTable.put(110.0, 3470.0);
    // rangeTable.put(115.0, 3000.0);
    rangeTable.put(120.0, 3770.0);
    // rangeTable.put(130.0, 3000.0);
    rangeTable.put(136.0, 4000.0);
    rangeTable.put(140.0, 4150.0); // guess
    // rangeTable.put(145.0, 3000.0);
    rangeTable.put(149.0, 4400.0); // guess

    DaisyExtensions.init();
    if (!m_debugMode) {
      SmartDashboard = NetworkTable.getTable("SmartDashboard");
    }
    downloadImages.setDefault(false);
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

    return 3150 + kRangeOffset;
  }

  public WPIImage processImage(WPIColorImage rawImage) {
    if (System.currentTimeMillis() > this.lastDownloadTime + this.downloadPeriod
        && this.downloadImages.getValue() == true) {
      try {
        BufferedImage image = rawImage.getBufferedImage();
        File outputfile = new File(
            "C:/Users/Miss Daisy/Pictures/MatchImages/Image(" + this.pictureCounter + ").jpg");
        ImageIO.write(image, "jpg", outputfile);
        this.lastDownloadTime = System.currentTimeMillis();
        pictureCounter++;
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
    double heading = 0.0;

    // Get the current heading of the robot first
    if (!m_debugMode) {
      try {
        heading = SmartDashboard.getNumber("heading", 0.0);
      } catch (NoSuchElementException | IllegalArgumentException e) {
      }
    }

    if (size == null || size.width() != rawImage.getWidth()
        || size.height() != rawImage.getHeight()) {
      size = opencv_core.cvSize(rawImage.getWidth(), rawImage.getHeight());
      bin = IplImage.create(size, 8, 1);
      hsv = IplImage.create(size, 8, 3);
      hue = IplImage.create(size, 8, 1);
      sat = IplImage.create(size, 8, 1);
      val = IplImage.create(size, 8, 1);
      horizontalOffsetPixels =
          (int) Math.round(pShooterOffsetDeg.getValue() * (size.width() / kHorizontalFOVDeg));
      // the points for the green line burned into the image
      linePt1 = new WPIPoint((size.width() / 2) + horizontalOffsetPixels, size.height() - 1);
      linePt2 = new WPIPoint((size.width() / 2) + horizontalOffsetPixels, 0);
    }
    // Get the raw IplImages for OpenCV
    IplImage input = DaisyExtensions.getIplImage(rawImage);

    // Convert to HSV color space
    opencv_imgproc.cvCvtColor(input, hsv, opencv_imgproc.CV_BGR2HSV);
    opencv_core.cvSplit(hsv, hue, sat, val, null);

    // Threshold each component separately
    // Hue
    // NOTE: Red is at the end of the color space, so you need to OR together
    // a thresh and inverted thresh in order to get points that are red
    /*
     * opencv_imgproc.cvThreshold(hue, bin, pHueLowerBound.getValue(), pHueUpperBound.getValue(),
     * opencv_imgproc.CV_THRESH_BINARY); opencv_imgproc.cvThreshold(hue, hue, 80, 120,
     * opencv_imgproc.CV_THRESH_BINARY_INV);
     */
    opencv_imgproc.cvThreshold(hue, bin, pHueLowerBound.getValue(), 255, opencv_imgproc.CV_THRESH_BINARY);
    opencv_imgproc.cvThreshold(hue, hue, pHueUpperBound.getValue(), 255, opencv_imgproc.CV_THRESH_BINARY_INV);

    // Saturation
    opencv_imgproc.cvThreshold(sat, sat, pSaturationLowerBound.getValue(), 255,
        opencv_imgproc.CV_THRESH_BINARY);

    // Value
    opencv_imgproc.cvThreshold(val, val, pValueLowerBound.getValue(), 255,
        opencv_imgproc.CV_THRESH_BINARY);

    // Combine the results to obtain our binary image which should for the
    // most
    // part only contain pixels that we care about
    opencv_core.cvAnd(bin, hue, bin, null);
    opencv_core.cvAnd(bin, sat, bin, null);
    opencv_core.cvAnd(bin, val, bin, null);

    // Uncomment the next two lines to see the raw binary image
   //  CanvasFrame result = new CanvasFrame("binary");
   // result.showImage(WPIImage.convertIplImageToBuffImage(bin));

    // Fill in any gaps using binary morphology
    opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE,
        kHoleClosingIterations);

    // Uncomment the next two lines to see the image post-morphology
    // CanvasFrame result2 = new CanvasFrame("morph");
    // result2.showImage(WPIImage.convertIplImageToBuffImage(bin));

    // Find contours
    WPIBinaryImage binWpi = DaisyExtensions.makeWPIBinaryImage(bin);

    // first, find all the convex contours
    contours = DaisyExtensions.findConvexContours(binWpi);
    targets = new ArrayList<Target>();

    for (WPIContour c : contours) {
      rawImage.drawPolygon(c.approxPolygon(1), WPIColor.YELLOW, 1);
      // I have yet to figure out why the second parameter needs to be > 0
      CvRect bound = opencv_imgproc.cvBoundingRect(c.getCVSeq(), 1);

      Target target = new Target();
      target.centroidX = bound.x() + (bound.width() / 2);
      target.centroidY = bound.y() + bound.height();
      target.width = bound.width();
      target.height = bound.height();
      target.countours = c;
      rawImage.drawRect(bound.x(), bound.y(), bound.width(), bound.height(), WPIColor.WHITE, 1);

      double kMinTargetWidth = 20;
      double kMaxTargetWidth = 300;
      double kMinTargetHeight = 6;
      double kMaxTargetHeight = 60;
      if (target.width < kMinTargetWidth || target.width > kMaxTargetWidth
          || target.height < kMinTargetHeight || target.height > kMaxTargetHeight) {
        continue;
      }

      // Filter based on shape
      double kMaxWideness = 7.0;
      double kMinWideness = 1.5;
      double wideness = target.width / target.height;
      if (wideness < kMinWideness || wideness > kMaxWideness) {
        continue;
      }

      // Filter based on fullness
      double kMinFullness = .45;
      double kMaxFullness = .95;
      double original_contour_area = opencv_imgproc.cvContourArea(c.getCVSeq());
      double area = target.width * target.height * 1.0;
      double fullness = original_contour_area / area;
      if (fullness < kMinFullness || fullness > kMaxFullness) {
        continue;
      }

      targets.add(target);
    }

    double kMaxOffset = 10;
    boolean found = false;
    for (int i = 0; !found && i < targets.size(); i++) {
      for (int j = 0; !found && j < targets.size(); j++) {
        if (i == j) {
          continue;
        }
        Target targetI = targets.get(i);
        Target targetJ = targets.get(j);
        double offset = Math.abs(targetI.centroidX - targetJ.centroidX);
        if (offset < kMaxOffset) {
          rawImage.drawContour(targets.get(i).countours, WPIColor.BLUE, 2);
          Target topTarget = targetI.centroidY > targetJ.centroidY ? targetI : targetJ;
          Target bottomTarget = targetI.centroidY < targetJ.centroidY ? targetI : targetJ;
          if (topTarget.height > bottomTarget.height) {
            targets.add(topTarget);
            found = true;
            break;
          }
        }
      }
    }

    // boolean readyToShoot = false;

    if (targets.size() > 0 && targets.get(targets.size() - 1) != null) {
      Target t = targets.get(targets.size() - 1);
      rawImage.drawPoint(new WPIPoint((int) t.centroidX, (int) t.centroidY - (int) (t.height / 2)),
          WPIColor.RED, 2);
      double x = t.centroidX;
      //x = (2 * (x / size.width())) - 1;

      double y = t.centroidY;
      y = -((2 * ((size.height() - y) / size.height())) - 1);

      double azimuth = heading + 
          -1 * Math.toDegrees(Math.atan((x - 319.5) / kFocalLength)) - pShooterOffsetDeg.getValue();
      double range = (kTargetHeightIn - kCameraHeightIn)
          / Math.tan((y * kVerticalFOVDeg / 2.0 + kCameraPitchDeg) * Math.PI / 180.0);
      double rpms = getRPMsForRange(range);

      if (!m_debugMode) {
        SmartDashboard.putBoolean("found", true);
        SmartDashboard.putNumber("azimuth", this.boundAngle0to360Degrees(azimuth));
        SmartDashboard.putNumber("rpms", rpms);
        SmartDashboard.putNumber("range", range);
        // readyToShoot = SmartDashboard.getBoolean("ReadyToShoot", false);
      } else {
        System.out.println("Target found");
        System.out.println("x: " + x);
        System.out.println("y: " + y);
        System.out.println("azimuth: " + this.boundAngle0to360Degrees(azimuth));
        System.out.println("range: " + range);
        System.out.println("rpms: " + rpms);
      }
      rawImage.drawContour(t.countours, WPIColor.GREEN, 3);
    } else {

      if (!m_debugMode) {
        SmartDashboard.putBoolean("found", false);
      } else {
        System.out.println("Target not found");
      }
    }

    if (!m_debugMode) {
      SmartDashboard.putNumber("axis timestamp", timestamp);
    }

    // Draw a crosshair
    rawImage.drawLine(linePt1, linePt2, targetColor, 2);
    DaisyExtensions.releaseMemory();

    return rawImage;
  }

  private double boundAngle0to360Degrees(double angle) {
    // Naive algorithm
    while (angle >= 360.0) {
      angle -= 360.0;
    }
    while (angle < 0.0) {
      angle += 360.0;
    }
    return angle;
  }

  public static double boundAngleNeg180to180Degrees(double angle) {
    // Naive algorithm
    while (angle >= 180.0) {
      angle -= 360.0;
    }
    while (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  @Override
  public void propertyChanged(Property arg0) {
    super.propertyChanged(arg0);
  }

  @Override
  public void init() {
    super.init();
    /*
     * if (Desktop.isDesktopSupported()) { try { Desktop.getDesktop().browse(new URI("http://" +
     * super.ipProperty.getValue())); } catch (IOException | URISyntaxException e) {
     * System.err.println("Could not open camera webpage."); } }
     */
  }

  public void drawGreenPolygon(IplImage image) {
    opencv_imgproc.rectangle(opencv_core.cvarrToMat(image), new opencv_core.Point(0, 0),
        new opencv_core.Point(50, 50), new opencv_core.Scalar(0, 255, 0, 0), 50, 8, 0);
  }

  public void drawRedPolygon(IplImage image) {
    opencv_core.Point pt1 = new opencv_core.Point(image.width() - 50, 1);
    opencv_core.Point pt2 = new opencv_core.Point(image.width(), 50);
    opencv_imgproc.rectangle(opencv_core.cvarrToMat(image), pt1, pt2,
        new opencv_core.Scalar(0, 0, 255, 0), 50, 0, 0);
  }

  public static void main(String[] args) {
    // if (args.length == 0) {
    // System.out.println("Usage: Arguments are paths to image files to test the program on");
    // return;
    // }

    // Create the widget
    DaisyCV widget = new DaisyCV(true);

    long totalTime = 0;

    File imgPath = new File("/Users/josh/Documents/LEDBoiler");
    System.out.println(imgPath.isDirectory());
    File[] imgs = imgPath.listFiles();
    for (int i = 0; i < imgs.length; i++) {
      // Load the image
      WPIColorImage rawImage = null;
      try {
        rawImage = new WPIColorImage(ImageIO.read(imgs[i]));
      } catch (IOException e) {
        System.err.println("Could not find file!" + args[i % args.length]);
        e.printStackTrace();
        return;
      }

      // shows the raw image before processing to eliminate the possibility
      // that both may be the modified image.
      CanvasFrame original = new CanvasFrame("Raw");
      original.showImage(rawImage.getBufferedImage());

      WPIImage resultImage = null;

      // Process image
      long startTime, endTime;
      startTime = System.nanoTime();
      resultImage = widget.processImage(rawImage);
      endTime = System.nanoTime();

      // Display results
      totalTime += (endTime - startTime);
      double milliseconds = (double) (endTime - startTime) / 1000000.0;
      System.out.format("Processing took %.2f milliseconds%n", milliseconds);
      System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);

      CanvasFrame result = new CanvasFrame("Result");
      result.showImage(resultImage.getBufferedImage());

      System.out.println("Waiting for ENTER to continue to next image or exit...");
      Scanner console = new Scanner(System.in);
      console.nextLine();

      if (original.isVisible()) {
        original.setVisible(false);
        original.dispose();
      }
      if (result.isVisible()) {
        result.setVisible(false);
        result.dispose();
      }
    }

    double milliseconds = (double) (totalTime) / 1000000.0 / (args.length);
    System.out.format("AVERAGE:%.2f milliseconds%n", milliseconds);
    System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);
    System.exit(0);
  }
}
