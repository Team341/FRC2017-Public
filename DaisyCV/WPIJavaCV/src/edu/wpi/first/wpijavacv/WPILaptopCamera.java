/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpijavacv;

import org.bytedeco.javacpp.opencv_videoio;
import org.bytedeco.javacpp.opencv_videoio.CvCapture;
/**
 * A class used to gather images from cameras connected to the laptop
 * @author Greg
 */
public class WPILaptopCamera extends WPIDisposable {
    CvCapture cam;

    public WPILaptopCamera() {
        cam = opencv_videoio.cvCreateCameraCapture(0);
    }

    public WPIColorImage getCurrentFrame(){
        return new WPIColorImage(opencv_videoio.cvQueryFrame(cam));
    }

    @Override
    protected void disposed() {
    }

}
