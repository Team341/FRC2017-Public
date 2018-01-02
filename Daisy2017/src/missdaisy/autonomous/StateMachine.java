package missdaisy.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Shooter;

/**
 * State machine to execute an autonomous mode.
 *
 * @author Jared341
 */
public class StateMachine {

  State[] mStates;
  int mCurrentState;
  boolean mStarted;

  public StateMachine(State[] aStates) {
    mStates = aStates;
    mCurrentState = 0;
    mStarted = false;
    // SmartDashboard.log("Not Started", "Autonomous Mode State");
  }

  public String getCurrentState() {
    if (mCurrentState < mStates.length) {
      return mStates[mCurrentState].toString();
    } else {
      return "Not Active";
    }
  }

  public void run() {
    if (mCurrentState < mStates.length) {
      SmartDashboard.putString("Current State:", mStates[mCurrentState].toString());
      if (!mStarted) {
        mStates[mCurrentState].enter();
        mStarted = true;
        System.out.println("Entering state: " + mStates[mCurrentState]);
        SmartDashboard.putString("CurrentAutoState", mStates[mCurrentState].toString());
        // "Autonomous Mode State");
      } else if (mStates[mCurrentState].isDone()) {
        mStates[mCurrentState].exit();
        System.out.println("Exiting state: " + mStates[mCurrentState]);
        SmartDashboard.putString("CurrentAutoStateExit", mStates[mCurrentState].toString());
        mCurrentState++;
        if (mCurrentState < mStates.length) {
          mStates[mCurrentState].enter();
          System.out.println("Entering state: " + mStates[mCurrentState]);
          SmartDashboard.putString("CurrentAutoState", mStates[mCurrentState].toString());
          // SmartDashboard.log(mStates[mCurrentState].toString(),
          // "Autonomous Mode State");
        } else {
          // SmartDashboard.log("Finished", "Autonomous Mode State");
          System.out.println("Finished state machine.");
          Drive.getInstance().setOpenLoop();
          Drive.getInstance().setSpeed(0.0, 0.0);
          Shooter.getInstance().setOpenLoop();
          Shooter.getInstance().setPercVoltage(0.0);
        }
      } else {
        mStates[mCurrentState].running();
      }
    }
  }
}
