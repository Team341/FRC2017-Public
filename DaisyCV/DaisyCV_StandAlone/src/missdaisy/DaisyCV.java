package missdaisy;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.TreeMap;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_core.CvSize;
import org.bytedeco.javacpp.opencv_core.IplConvKernel;
import org.bytedeco.javacpp.opencv_core.IplImage;
import org.bytedeco.javacpp.opencv_imgproc;
import org.bytedeco.javacv.CanvasFrame;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpijavacv.*;
import fileio.PropertySet;

/* HOW TO GET THIS COMPILING IN NETBEANS:

 *  1. Install the SmartDashboard using the installer (if on Windows)
 *      1a. Verify that the OpenCV libraries are in your PATH (on Windows)
 *  2. Add the following libraries to the project:
 *     SmartDashboard.jar
 *     extensions/WPICameraExtension.jar
 *     lib/NetworkTable_Client.jar
 *     extensions/lib/javacpp.jar
 *     extensions/lib/javacv-*your environment*.jar
 *     extensions/lib/javacv.jar
 *     extensions/lib/WPIJavaCV.jar
 *
 */
/**
 *
 * @author jrussell
 */
public class DaisyCV {
	public static final String NAME = "DaisyCV 2016";
	private WPIColor targetColor = new WPIColor(0, 255, 0);
	private NetworkTable SmartDashboard;
	private PropertySet properties;

	// Constants that need to be tuned
	// basically, the horizontal slope is the acceptable tilt of the lines found
	private static final double kNearlyHorizontalSlope = Math.tan(Math.toRadians(20));
	private static final double kNearlyVerticalSlope = Math.tan(Math.toRadians(90 - 20));
	private static final int kMinWidth = 20;
	private static final int kMaxWidth = 200;
	private static final double kRangeOffset = 0.0;
	private static final int kHoleClosingIterations = 2;

	private static final double kShooterOffsetDeg = 1.0;
	private static final double kHorizontalFOVDeg = 47.0;

	private static final double kVerticalFOVDeg = 480.0 / 640.0 * kHorizontalFOVDeg;
	private static final double kCameraHeightIn = 38.5;
	private static final double kCameraPitchDeg = 18.8;
	private static final double kTargetHeightIn = 82.0 + 13; // 82 to bottom of target, 13 to middle to bottom of target
	// p for properties, so that editing the program can be done on the fly
	// they are initialized to default values, and these values are used if 
	// a properties file cannot be found
	private double pHueLowerBound = 55;
	private double pHueUpperBound = 95;
	private double pSaturationLowerBound = 170;
	private double pSaturationUpperBound = 255;
	private double pValueLowerBound = 89;
	private double pValueUpperBound = 255;
																
	private TreeMap<Double, Double> rangeTable;

	private boolean m_debugMode = false;

	// Store JavaCV temporaries as members to reduce memory management during
	// processing
	private CvSize size = null;
	private WPIContour[] contours;
	private ArrayList<WPIPolygon> polygons;
	private IplConvKernel morphKernel;
	private IplImage bin;
	private IplImage hsv;
	private IplImage hue;
	private IplImage sat;
	private IplImage val;
	private WPIPoint linePt1;
	private WPIPoint linePt2;
	private int horizontalOffsetPixels;

	public DaisyCV() {
		this(false);
	}

	public DaisyCV(boolean debug) {
		m_debugMode = debug;
		morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);
		properties = PropertySet.getInstance();
				
		// gets these values from the properties file that must be created
		pHueLowerBound = properties.getDoubleValue("HueLowerBound", pHueLowerBound);
		pHueUpperBound = properties.getDoubleValue("HueUpperBound", pHueUpperBound);
		pSaturationLowerBound = properties.getDoubleValue("SaturationLowerBound", pSaturationLowerBound);
		pSaturationUpperBound = properties.getDoubleValue("SaturationUpperBound", pSaturationUpperBound);
		pValueLowerBound = properties.getDoubleValue("ValueLowerBound", pValueLowerBound);
		pValueUpperBound = properties.getDoubleValue("ValueUpperBound", pValueUpperBound);

		rangeTable = new TreeMap<Double, Double>();
		rangeTable.put(150.0, 6375.0 + kRangeOffset);
		rangeTable.put(141.0, 6250.0 + kRangeOffset);
		
		DaisyExtensions.init();
		if (!m_debugMode) {
			NetworkTable.setClientMode();
			NetworkTable.setIPAddress("roborio-341-frc.local");
			SmartDashboard = NetworkTable.getTable("SmartDashboard");
		}
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

		return 4000 + kRangeOffset;
	}

	public WPIImage processImage(WPIColorImage rawImage) {
		double heading = 0.0;

		// Get the current heading of the robot first
		if (!m_debugMode) {
			try { heading = SmartDashboard.getNumber("heading", 0.0); }
			catch (NoSuchElementException | IllegalArgumentException e) { }
		}

		if (size == null || size.width() != rawImage.getWidth() || size.height() != rawImage.getHeight()) {
			size = opencv_core.cvSize(rawImage.getWidth(), rawImage.getHeight());
			bin = IplImage.create(size, 8, 1);
			hsv = IplImage.create(size, 8, 3);
			hue = IplImage.create(size, 8, 1);
			sat = IplImage.create(size, 8, 1);
			val = IplImage.create(size, 8, 1);
			horizontalOffsetPixels = (int) Math.round(kShooterOffsetDeg * (size.width() / kHorizontalFOVDeg));
			// the points for the green line burned into the image
			linePt1 = new WPIPoint(size.width() / 2, size.height() - 1);
			linePt2 = new WPIPoint(size.width() / 2, 0);
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
		opencv_imgproc.cvThreshold(hue, bin, pHueLowerBound, pHueUpperBound, opencv_imgproc.CV_THRESH_BINARY);
		//opencv_imgproc.cvThreshold(hue, hue, 60 + 15, 255, opencv_imgproc.CV_THRESH_BINARY_INV);

		// Saturation
		opencv_imgproc.cvThreshold(sat, sat, pSaturationLowerBound, pSaturationUpperBound, opencv_imgproc.CV_THRESH_BINARY);

		// Value
		opencv_imgproc.cvThreshold(val, val, pValueLowerBound, pValueUpperBound, opencv_imgproc.CV_THRESH_BINARY);

		// Combine the results to obtain our binary image which should for the
		// most
		// part only contain pixels that we care about
		opencv_core.cvAnd(bin, hue, bin, null);
		opencv_core.cvAnd(bin, sat, bin, null);
		opencv_core.cvAnd(bin, val, bin, null);

		// Uncomment the next two lines to see the raw binary image
		// CanvasFrame result = new CanvasFrame("binary");
		// result.showImage(WPIImage.convertIplImageToBuffImage(bin));

		// Fill in any gaps using binary morphology
		opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE, kHoleClosingIterations);

		// Uncomment the next two lines to see the image post-morphology
		// CanvasFrame result2 = new CanvasFrame("morph");
		// result2.showImage(WPIImage.convertIplImageToBuffImage(bin));

		// Find contours
		WPIBinaryImage binWpi = DaisyExtensions.makeWPIBinaryImage(bin);
		contours = DaisyExtensions.findConvexContours(binWpi);

		polygons = new ArrayList<WPIPolygon>();
		for (WPIContour c : contours) {
			double ratio = ((double) c.getHeight()) / ((double) c.getWidth());
			if (c.getHeight() * c.getWidth() > 100) {
        System.out.println("Ratio: " + ratio);
        if (ratio < .6 && ratio > 0.1 && c.getWidth() > kMinWidth && c.getWidth() < kMaxWidth) {
          polygons.add(c.approxPolygon(0));
        }
      }
		}

		WPIPolygon square = null;
		int largest = Integer.MIN_VALUE;

		for (WPIPolygon p : polygons) {
			if (p.isConvex()) {
				// We passed the first test...we fit a rectangle to the polygon
				// Now do some more tests

				WPIPoint[] points = p.getPoints();
				System.out.println("Points length" + points.length);
				// We expect to see a top line that is nearly horizontal, and
				// two side lines that are nearly vertical
				int numNearlyHorizontal = 0;
				int numNearlyVertical = 0;
				for (int i = 0; i < points.length; i++) {
					double dy = points[i].getY() - points[(i + 1) % 4].getY();
					double dx = points[i].getX() - points[(i + 1) % 4].getX();
					double slope = Double.MAX_VALUE;
					if (dx != 0)
						slope = Math.abs(dy / dx);

					if (slope < kNearlyHorizontalSlope)
						++numNearlyHorizontal;
					else if (slope > kNearlyVerticalSlope)
						++numNearlyVertical;
				}
				
				if (numNearlyHorizontal >= 1 && numNearlyVertical >=1) {
					rawImage.drawPolygon(p, WPIColor.BLUE, 2);

					int pCenterX = (p.getX() + (p.getWidth() / 2));
					int pCenterY = (p.getY());
					int pArea = (p.getArea());

					rawImage.drawPoint(new WPIPoint(pCenterX, pCenterY), WPIColor.RED, 5);
					// because coord system is funny( largest y value is at
					// bottom of picture)

					if (pArea > largest) {
						square = p;
						largest = pArea;
					}
				}
			} else {
				rawImage.drawPolygon(p, WPIColor.YELLOW, 1);
			}
		}
		//boolean readyToShoot = false;

		if (square != null) {
			double x = square.getX() + (square.getWidth() / 2);
			x = (2 * (x / size.width())) - 1;

			double y = square.getY();
			y = -((2 * (y / size.height())) - 1);

			double azimuth = this.boundAngle0to360Degrees(x * kHorizontalFOVDeg / 2.0 + heading - kShooterOffsetDeg);
			double range = (kTargetHeightIn - kCameraHeightIn)
					/ Math.tan((y * kVerticalFOVDeg / 2.0 + kCameraPitchDeg) * Math.PI / 180.0);
			double rpms = getRPMsForRange(range);

			if (!m_debugMode) {
				SmartDashboard.putBoolean("found", true);
				SmartDashboard.putNumber("azimuth", azimuth);
				SmartDashboard.putNumber("rpms", rpms);
				SmartDashboard.putNumber("range", range);
				//readyToShoot = SmartDashboard.getBoolean("ReadyToShoot", false);
			} else {
				System.out.println("Target found");
				System.out.println("x: " + x);
				System.out.println("y: " + y);
				System.out.println("azimuth: " + azimuth);
				System.out.println("range: " + range);
				System.out.println("rpms: " + rpms);
			}
			rawImage.drawPolygon(square, targetColor, 7);
		} else {

			if (!m_debugMode) {
				SmartDashboard.putBoolean("found", false);
			} else {
				System.out.println("Target not found");
			}
		}
		
		// Draw a crosshair
		rawImage.drawLine(linePt1, linePt2, targetColor, 2);
		DaisyExtensions.releaseMemory();
	
		return rawImage;
	}
	
	public WPIImage convertToBinary(WPIColorImage rawImage) {
		if (size == null || size.width() != rawImage.getWidth() || size.height() != rawImage.getHeight()) {
			size = opencv_core.cvSize(rawImage.getWidth(), rawImage.getHeight());
			bin = IplImage.create(size, 8, 1);
			hsv = IplImage.create(size, 8, 3);
			hue = IplImage.create(size, 8, 1);
			sat = IplImage.create(size, 8, 1);
			val = IplImage.create(size, 8, 1);
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
		opencv_imgproc.cvThreshold(hue, bin, pHueLowerBound, pHueUpperBound, opencv_imgproc.CV_THRESH_BINARY);
		//opencv_imgproc.cvThreshold(hue, hue, 40, 255, opencv_imgproc.CV_THRESH_BINARY_INV);

		// Saturation
		opencv_imgproc.cvThreshold(sat, sat, pSaturationLowerBound, pSaturationUpperBound, opencv_imgproc.CV_THRESH_BINARY);

		// Value
		opencv_imgproc.cvThreshold(val, val, pValueLowerBound, pValueUpperBound, opencv_imgproc.CV_THRESH_BINARY);

		// Combine the results to obtain our binary image which should for the
		// most
		// part only contain pixels that we care about
		opencv_core.cvAnd(bin, hue, bin, null);
		opencv_core.cvAnd(bin, sat, bin, null);
		opencv_core.cvAnd(bin, val, bin, null);

		return new WPIImage(bin);
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
}
