package edu.wpi.first.wpijavacv;


import org.bytedeco.javacpp.opencv_core.IplImage;
import org.bytedeco.javacv.FrameGrabber;
import org.bytedeco.javacv.OpenCVFrameConverter.ToIplImage;
import org.bytedeco.javacv.OpenCVFrameGrabber;

import static org.bytedeco.javacpp.opencv_core.cvCopy;
import static org.bytedeco.javacpp.opencv_core.cvCreateImage;

/**
 * A class used to gather images from the driver station camera.
 *
 * @author Joe Grinstead and Greg Granito
 */
public class WPIWebcam extends WPIDisposable {

    private final Object imageLock = new Object();
    private final Object grabberLock = new Object();
    private OpenCVFrameGrabber grabber;
    private IplImage image;
    private boolean readImage = true;
    private boolean badConnection = false;

    public WPIWebcam() {
        final Thread thread = new Thread() {
            @Override
            public void run() {
                grabber = new OpenCVFrameGrabber("");
                grabber.setFrameRate(1.0);
                try {
                    grabber.start();

                    while (!isDisposed()) {
                        try {
                            IplImage newest;
                            synchronized (grabberLock) {
                                if (isDisposed()) {
                                    return;
                                }
                                ToIplImage toIplImage = new ToIplImage();
                                newest = toIplImage.convert(grabber.grab());
                            }
                            if (isNull(newest)) {
                                synchronized (imageLock) {
                                    badConnection = true;
                                    imageLock.notify();
                                }
                                return;
                            } else {
                                synchronized (imageLock) {
                                    if (image == null) {
                                        image = cvCreateImage(newest.cvSize(), newest.depth(), newest.nChannels());
                                    }
                                    cvCopy(newest, image);
                                    readImage = false;
                                    badConnection = false;
                                    imageLock.notify();
                                }
                            }
                        } catch (Exception ex) {
                            synchronized (imageLock) {
                                badConnection = true;
                                imageLock.notify();
                            }
                            ex.printStackTrace();
                            return;
                        }
                        try {
                            Thread.sleep(20);
                        } catch (InterruptedException ex) {
                        }
                    }
                } catch (Exception ex) {
                    synchronized (imageLock) {
                        badConnection = true;
                        imageLock.notify();
                    }
                    if ((ex instanceof FrameGrabber.Exception) && ex.getMessage().startsWith("avformat_open_input() error -5")) {
                        System.err.println("* FrameGrabber.Exception: " + ex.getMessage().substring(0, ex.getMessage().indexOf('"')));
                    } else ex.printStackTrace();
                }
            }
        };
        thread.setDaemon(true);
        thread.start();
    }

    public WPIImage getImage() throws BadConnectionException {
        validateDisposed();

        synchronized (imageLock) {
            if (badConnection) {
                throw new BadConnectionException();
            } else if (image == null) {
                return null;
            } else if (image.nChannels() == 1) {
                return new WPIGrayscaleImage(image.clone());
            } else {
                assert image.nChannels() == 3;
                return new WPIColorImage(image.clone());
            }
        }
    }

    public WPIImage getNewImage(double timeout) throws BadConnectionException {
        validateDisposed();

        synchronized (imageLock) {
            readImage = true;
            while (readImage && !badConnection) {
                try {
                    badConnection = true;
                    imageLock.wait((long) (timeout * 1000));
                } catch (InterruptedException ex) {
                }
            }
            readImage = true;

            if (badConnection) {
                throw new BadConnectionException();
            } else if (image.nChannels() == 1) {
                return new WPIGrayscaleImage(image.clone());
            } else {
                assert image.nChannels() == 3;
                return new WPIColorImage(image.clone());
            }
        }
    }

    public WPIImage getNewImage() throws BadConnectionException {
        return getNewImage(0);
    }

    @Override
    protected void disposed() {
        try {
            synchronized (imageLock) {
                if (!isNull(image)) {
                    image.release();
                }
                image = null;
            }
        } catch (Exception ex) {
        }
    }

    @Override
    protected void finalize() throws Throwable {
        grabber.stop();
        super.finalize();
    }

    /**
     * An exception that occurs when the camera can not be reached.
     *
     * @author Greg Granito
     */
    public static class BadConnectionException extends Exception {
    }
}
