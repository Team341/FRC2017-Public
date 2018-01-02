/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpijavacv;

import java.awt.image.BufferedImage;
import org.bytedeco.javacpp.opencv_core.IplImage;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.Java2DFrameConverter;
import org.bytedeco.javacv.OpenCVFrameConverter;

/**
 * This class is the superclass of all images.
 * 
 * @author Greg Granito
 */
public class WPIImage extends WPIDisposable {

    OpenCVFrameConverter.ToIplImage converter = new OpenCVFrameConverter.ToIplImage();

	
    /** The underlying {@link IplImage} */
    public IplImage image;

    /**
     * Instantiates a {@link WPIImage} from a {@link BufferedImage}.
     * Useful for interacting with swing.
     * This will not keep a reference to the given image, so any modification to this
     * {@link WPIImage} will not change the given image.
     * @param image the image to copy into a {@link WPIImage}
     */
 

    /**
     * Instantiates a {@link WPIImage} from the given {@link IplImage}.
     * The resulting image will be directly wrapped around the given image, and so any modifications
     * to the {@link WPIImage} will reflect on the given image.
     * @param image the image to wrap
     */
    public WPIImage(IplImage image) {
        this.image = image;
    }

    public WPIImage(BufferedImage image) {
    	this.image = convertBuffToIplImage(image);
	}

	/**
     * Returns the width of the image.
     * @return the width in pixels of the image
     */
    public int getWidth() {
        validateDisposed();

        return image.width();
    }

    /**
     * Returns the height of the image.
     * @return the height in pixels of the image
     */
    public int getHeight() {
        validateDisposed();

        return image.height();
    }

    /**
     * Copies this {@link WPIImage} into a {@link BufferedImage}.
     * This method will always generate a new image.
     * @return a copy of the image
     */
    public BufferedImage getBufferedImage() {
        validateDisposed();

        return convertIplImageToBuffImage(image);
    }

    @Override
    protected void disposed() {
        image.release();
    }
    public static BufferedImage convertIplImageToBuffImage(IplImage src) {
        OpenCVFrameConverter.ToIplImage grabberConverter = new OpenCVFrameConverter.ToIplImage();
        Java2DFrameConverter paintConverter = new Java2DFrameConverter();
        Frame frame = grabberConverter.convert(src);
        return paintConverter.getBufferedImage(frame,1);
    }
    
	public static IplImage convertBuffToIplImage(BufferedImage img){
	         Java2DFrameConverter converter1 = new Java2DFrameConverter();
	         OpenCVFrameConverter.ToIplImage converter2 = new OpenCVFrameConverter.ToIplImage();
	         IplImage iploriginal = converter2.convert(converter1.convert(img));
	         return iploriginal.clone();
	}
	
	public IplImage getIplImage() {
    	return image;
    }
}

