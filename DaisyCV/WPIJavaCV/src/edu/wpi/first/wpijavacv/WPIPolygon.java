/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpijavacv;

import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_core.CvPoint;
import org.bytedeco.javacpp.opencv_core.CvRect;
import org.bytedeco.javacpp.opencv_core.CvSeq;
import org.bytedeco.javacpp.opencv_imgproc;

/**
 * A class that represents a polygon, can be obtained from approxPolygon()
 * in WpiContour
 * @author Greg Granito
 */
public class WPIPolygon extends WPIDisposable {

    CvSeq polygon;
    CvRect boundingRect;

    public WPIPolygon(CvSeq data) {
        polygon = data;
    }

    CvSeq getCVSeq() {
        return polygon;
    }

    /**
     *
     * @return an array of WpiPoints of the vertices of the polygon
     */
    public WPIPoint[] getPoints() {
        CvPoint points = new CvPoint(getNumVertices());
        WPIPoint[] wpiPoints= new WPIPoint[getNumVertices()];
        opencv_core.cvCvtSeqToArray(polygon, points.position(0), opencv_core.CV_WHOLE_SEQ);
        for (int j = 0; j < getNumVertices(); j++) {
            
            wpiPoints[j] = new WPIPoint(points.position(j).x(), points.position(j).y());
            
        }
        try {
			points.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
        return wpiPoints;
    }

    /**
     *
     * @return the width of the bounding rectangle of the polygon
     */
    public int getWidth() {
        if (boundingRect == null) {
            boundingRect = opencv_imgproc.cvBoundingRect(polygon, 0);
        }
        return boundingRect.width();
    }

    /**
     *
     * @return the height of the bounding rectangle of the polygon
     */
    public int getHeight() {
        if (boundingRect == null) {
            boundingRect = opencv_imgproc.cvBoundingRect(polygon, 0);
        }
        return boundingRect.height();
    }

    /**
     *
     * @return the x coord of the top left corner of the bounding
     * rectangle of the polygon
     */
    public int getX() {
        if (boundingRect == null) {
            boundingRect = opencv_imgproc.cvBoundingRect(polygon, 0);
        }
        return boundingRect.x();
    }

    /**
     *
     * @return the y coord of the top left corner of the bounding
     * rectangle of the polygon
     */
    public int getY() {
        if (boundingRect == null) {
            boundingRect = opencv_imgproc.cvBoundingRect(polygon, 0);
        }
        return boundingRect.y();
    }

    /**
     *
     * @return the number of vertices in the polygon
     */
    public int getNumVertices() {
        return polygon.total();

    }

    /**
     *
     * @return whether or not the polygon is convex
     */
    public boolean isConvex() {
        return opencv_imgproc.cvCheckContourConvexity(polygon) == 0 ? false : true;
    }

    /**
     *
     * @return the area in pixels of the polygon
     */
    public int getArea() {
        return Math.abs((int) opencv_imgproc.cvContourArea(polygon, opencv_core.CV_WHOLE_SEQ, -1));
    }

    /**
     *
     * @return the perimeter in pixels of the polygon
     */
    public int getPerimeter() {
        return (int) opencv_imgproc.cvArcLength(polygon, opencv_core.CV_WHOLE_SEQ, -1);
    }

    public void disposed() {
        polygon.deallocate();
    }
}
