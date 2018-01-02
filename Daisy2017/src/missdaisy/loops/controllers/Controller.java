package missdaisy.loops.controllers;

/**
 * Interface for a generic Controller.
 *
 * A controller is something that attempts to control a subsystem to get to a specified goal.
 *
 * @author Jared341
 */
public interface Controller extends Runnable {

  public void reset();

  public boolean onTarget();

  public void loadProperties();
}
