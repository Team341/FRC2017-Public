package missdaisy.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import missdaisy.fileio.PropertySet;
import missdaisy.loops.controllers.Controller;

/**
 * Base class for a subsystem.
 *
 * A subsystem is a mechanism or cohesive function of a robot. Examples could be: Drive, Arm,
 * Shooter, Intake, etc.
 *
 * @author Jared341
 */
public abstract class DaisySubsystem extends Subsystem {

  protected Controller mCurrentController;
  protected PropertySet mPropertySet = PropertySet.getInstance();

  @Override
  public void initDefaultCommand() {
    
  }

  public synchronized Controller getCurrentController() {
    return mCurrentController;
  }

  public synchronized void setCurrentController(Controller controller) {
    mCurrentController = controller;
  }

  public synchronized void setOpenLoop() {
    mCurrentController = null;
  }

  public synchronized void runCurrentController() {
    if (mCurrentController != null) {
      mCurrentController.run();
    }
  }

  public synchronized void runOutputFilters() {

  }

  public synchronized void runInputFilters() {

  }

  public synchronized void reset() {

  }

  public synchronized void loadProperties() {

  }
}
