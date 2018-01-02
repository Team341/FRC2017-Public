package missdaisy;

import java.awt.Desktop;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;
import java.util.Scanner;
import javax.imageio.ImageIO;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;
import fileio.PropertyReader;
import fileio.PropertySet;

/**
 * Connects to the axis camera's IP and retrieves the latest frame
 * Processes the frame to output data to smartdashboard
 * such as range, azimuth, rpm, etc.
 * 
 * @author Josh Sizer
 */

public class Main {
	// the ip of the axis camera
	private static final String kCameraIP = "http://axis-camera.local/";
	private static int mMode = 1; // 0 for normal use, 1 for debug, 2 for
								  // downloading images, 3 for seeing live 
								  // binary images
	public static void main(String[] args) {
		//PropertyReader mReader = new PropertyReader();
		//mReader.parseFile("C:/Users/" + "Miss Daisy" + "/Desktop/HSVProperties.txt");
		DaisyCV imageProcessor;
		if (mMode == 1)
			imageProcessor = new DaisyCV(true);
		else
			imageProcessor = new DaisyCV(false);
		DaisyGUI GUI = DaisyGUI.getInstance();
		WPIColorImage rawImage = null;
		WPIImage resultImage = null;
		URL cameraURL = null;
		URLConnection cameraConnection;
		mMode = PropertySet.getInstance().getIntValue("Mode", mMode);
		Scanner console = new Scanner(System.in);

		if (mMode == 0) {
			// open the camera's web browser's view because it makes everything
			// silky smooth.
			if (Desktop.isDesktopSupported()) {
				try {
					Desktop.getDesktop().browse(new URI(kCameraIP));
				} catch (IOException | URISyntaxException e) {
					System.err.println("Could not open camera webpage.");
				}
			}
			
			while (true) {
				try {
					cameraURL = new URL(kCameraIP + "axis-cgi/jpg/image.cgi");
					cameraConnection = cameraURL.openConnection();
					// grab the latest frame
					rawImage = new WPIColorImage(ImageIO.read(cameraConnection.getInputStream())); 
					if (rawImage != null) {
						resultImage = imageProcessor.processImage(rawImage); // process image
						GUI.showOutputImage(resultImage.getBufferedImage()); // show processed image
					}
				} catch (IOException e) {
					System.err.println("Could not connect to axis camera");
					//GUI.showError();
				}
				rawImage = null;
				resultImage = null;
			}
		} else if (mMode == 1) {
			if (args.length == 0) {
				System.out.println("Usage: Arguments are paths to image files to test the program on");
				return;
			}

			long totalTime = 0;
			for (int i = 0; i < args.length; i++) {
				// Load the image
				rawImage = null;
				try {
					rawImage = new WPIColorImage(ImageIO.read(new File(args[i % args.length])));
				} catch (IOException e) {
					System.err.println("Could not find file!");
					return;
				}

				// shows the raw image before processing to eliminate the
				// possibility that both may be the modified image.
				//GUI.showInputImage(rawImage.getBufferedImage());

				resultImage = null;

				// Process image
				long startTime, endTime;
				startTime = System.nanoTime();
				resultImage = imageProcessor.processImage(rawImage);
				endTime = System.nanoTime();

				// Display results
				totalTime += (endTime - startTime);
				double milliseconds = (double) (endTime - startTime) / 1000000.0;
				System.out.format("Processing took %.2f milliseconds%n", milliseconds);
				System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);

				GUI.showOutputImage(resultImage.getBufferedImage());

				System.out.println("Waiting for ENTER to continue to next image or exit...");
				console.nextLine();
			}
			
			double milliseconds = (double) (totalTime) / 1000000.0 / (args.length);
			System.out.format("AVERAGE:%.2f milliseconds%n", milliseconds);
			System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);
			System.exit(0);
		} else if (mMode == 2) {
			int numImages = 0;
			int currentImageNum = 0;
			File destination = null;
			BufferedImage currentImage = null;
			
			// exits the program if there are no arguments
			if (args.length == 0) {
				System.err.println("Usage: Arguments are the number of images to save followed by the desired location to save to.");
				return;
			}
			
			// exits the program if the first argument is not an integer
			try {
				numImages = Integer.parseInt(args[0]);
			} catch (NumberFormatException e) {
				System.err.println("First argument is the number of images to be saved.");
				return;
			}
			
			// saves the specified number of images
			for (int i = 0; i < numImages; i++) {		
				// grab the latest frame, or exit the program if it cannot connect to the camera
				try {
					cameraURL = new URL(kCameraIP + "axis-cgi/jpg/image.cgi");
					cameraConnection = cameraURL.openConnection();
					currentImage = ImageIO.read(cameraConnection.getInputStream());
				} catch (IOException e) {
					System.err.println("Could not connect to the camera.");
					return;
				}	
	
				// file path is the runtime argument + 2016TestImages/Image (number).jpg
				destination = new File(args[1], String.format("2016TestImages/Image(%d).jpg", currentImageNum));
				if (!destination.exists()) { // if the file path doesn't exist,
					destination.mkdirs(); // create the path
				} else {
					while (destination.exists()) { // if it does exist, increment image number
						currentImageNum++;
						destination = new File(args[1], String.format("2016TestImages/Image(%d).jpg", currentImageNum));
					}
				}
				
				// write the current image to disk or exit the program if we cannot write to the destination
				try {
					ImageIO.write(currentImage, "jpg", destination); // write the image to disk
				} catch (IOException e) {
					System.err.println("Could not write to destination"); 
					return;
				}
			}
			// %d is a decimal number (a base 10 number), %s is a string
			System.out.format("Saved %d images to %s.", numImages, destination.getParent());
			System.exit(0);
		} else {
			// open the camera's web browser's view because it makes everything
			// silky smooth.
			if (Desktop.isDesktopSupported()) {
				try {
					Desktop.getDesktop().browse(new URI(kCameraIP));
				} catch (IOException | URISyntaxException e) {
					System.err.println("Could not open camera webpage.");
				}
			}
			
			while (true) {
				try {
					cameraURL = new URL(kCameraIP + "axis-cgi/jpg/image.cgi");
					cameraConnection = cameraURL.openConnection();
					// grab the latest frame
					rawImage = new WPIColorImage(ImageIO.read(cameraConnection.getInputStream())); 
					if (rawImage != null) {
						GUI.showInputImage(rawImage.getBufferedImage()); // show input
						resultImage = imageProcessor.convertToBinary(rawImage); // process image
						GUI.showOutputImage(resultImage.getBufferedImage()); // show processed image
					}
				} catch (IOException e) {
					System.err.println("Could not connect to axis camera");
					GUI.showError();
				}
				rawImage = null;
				resultImage = null;
			}
		}
	}
}