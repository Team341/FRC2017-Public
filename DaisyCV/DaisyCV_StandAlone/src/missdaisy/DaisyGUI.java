package missdaisy;

import java.awt.image.BufferedImage;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import org.bytedeco.javacv.CanvasFrame;

public class DaisyGUI {
	private static DaisyGUI instance = null;
	private CanvasFrame inputWindow;
	private CanvasFrame outputWindow;
	
	public static DaisyGUI getInstance() {
		if (instance == null) 
			instance = new DaisyGUI();
		return instance;
	}
	
	private DaisyGUI() {
		inputWindow = new CanvasFrame("Input");
		outputWindow = new CanvasFrame("Output");
		inputWindow.setSize(640, 480);
		outputWindow.setSize(640, 480);
		
		inputWindow.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		outputWindow.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		
		// this is how you exit the program
		inputWindow.addWindowListener(new java.awt.event.WindowAdapter() {
			@Override
			public void windowClosing(java.awt.event.WindowEvent windowEvent) {
				if (JOptionPane.showConfirmDialog(inputWindow, "Are you sure to exit the program?", "Exit?",
						JOptionPane.YES_NO_OPTION, JOptionPane.QUESTION_MESSAGE) == JOptionPane.YES_OPTION) {
					
					if (inputWindow.isVisible()) {
						inputWindow.setVisible(false);
						inputWindow.dispose();
					}
					if (outputWindow.isVisible()) {
						outputWindow.setVisible(false);
						outputWindow.dispose();
					}
					System.exit(0);
				}
			}
		});
		
		outputWindow.addWindowListener(new java.awt.event.WindowAdapter() {
			@Override
			public void windowClosing(java.awt.event.WindowEvent windowEvent) {
				if (JOptionPane.showConfirmDialog(outputWindow, "Are you sure to exit the program?", "Exit?",
						JOptionPane.YES_NO_OPTION, JOptionPane.QUESTION_MESSAGE) == JOptionPane.YES_OPTION) {
					
					if (inputWindow.isVisible()) {
						inputWindow.setVisible(false);
						inputWindow.dispose();
					}
					if (outputWindow.isVisible()) {
						outputWindow.setVisible(false);
						outputWindow.dispose();
					}
					System.exit(0);
				}
			}
		}); }
	
	public void showInputImage(BufferedImage image) {
		inputWindow.showImage(image);		
	}
	
	public void showOutputImage(BufferedImage image) {
		outputWindow.showImage(image);
	}
	
	public void showError() {
		JOptionPane.showMessageDialog(inputWindow, "Could not connect to axis camera! Click ok to exit.", "Error!", 0);
	}
}
