package missdaisy.test;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;


/**
 * Test harness to read in a trajectory file and follow it using the PathFinder tool by Jaci
 * https://github.com/JacisNonsense/Pathfinder
 *
 * @author AJN
 */


public class LoadTrajectoryFromCSV {

  public static void main(String[] args) {

    // Load in the file with the trajectory
    File myFile = new File(
        "C:/Users/ajn5049/workspace/Daisy2017/src/config/Auton Trajectories/myOtherFile.csv");
    Trajectory trajectory = Pathfinder.readFromCSV(myFile);

    // Convert the trajectory for use by a tank drive
    TankModifier modifier = new TankModifier(trajectory).modify(2.6);  // set the robot width

    // Create 2 encoder followers that will be used to follow the left and right paths
    EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());

    // Configure the followers
    left.configureEncoder(0, 255, 4.0);
    right.configureEncoder(0, 255, 4.0);

    // Configure the PID/VA gains
    left.configurePIDVA(1.0, 0.0, 0.0, 1.0 / 7.0, 0.0);
    right.configurePIDVA(1.0, 0.0, 0.0, 1.0 / 7.0, 0.0);

    // Get the generated command at various points along the trajectory
    int[] encoderValues = {0, 255, 255 * 3, 255 * 6, 255 * 15, 255 * 30,
        255};    //(i.e. 0, 4, 12, 24 inches)
    for (int i = 0; i < encoderValues.length; i++) {
      double l = left.calculate(encoderValues[0]);
      double r = right.calculate(encoderValues[0]);

      double gyro_heading = 0.0; // Assuming the gyro is giving a value in
      // degrees
      double desired_heading = Pathfinder.r2d(left.getHeading()); // Should
      // also
      // be in
      // degrees

      double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
      double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

      //setLeftMotors(l + turn);
      //setRightMotors(r - turn);

      System.out.println("Command @ encoder value " + encoderValues[i] + ": " + l);
    }

  }

}
