package missdaisy.autonomous;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Type;
import missdaisy.fileio.PropertySet;

/**
 * Parses autonomous modes from an auto mode text file.
 *
 * Uses reflection to create instances of each auto state based on its name If it cannot be
 * instantiated for various reasons, WaitForTime(0) is instantiated
 *
 * @author AJN, Joshua Sizer
 */
public class AutonomousParser {

  private final PropertySet mProperties;
  private Class<?> currentState;
  private Constructor<?> mStateConstructor;
  private Type[] mParamTypes;
  int mNumberOfParameters;

  public AutonomousParser() {
    mProperties = PropertySet.getInstance();
  }

  public State[] parseStates(boolean mIsRedSide) {
    State[] fullAuto;

    fullAuto = parseStates();

    /*
     * System.out.println("Searching for selected auto,  "); switch (mStartPosition) { case 0: //
     * Near the Boiler System.out.println("Auto Selected: Near Boiler,  "); fullAuto = new
     * State[]{}; break; case 1: // Middle of alliance wall System.out.println(
     * "Auto Selected: Middle,  "); fullAuto = new State[]{}; break; case 2: // Far from Boiler
     * System.out.println("Auto Selected: Far from Boiler,  "); fullAuto = parseStates(); break;
     * default: System.out.println("Auto Selected: Do Nothing,  "); fullAuto = new State[]{}; }
     */

    for (State curState : fullAuto) {
      if (curState == null) {
        curState = new WaitForTime(0);
      }
    }

    return fullAuto;
  }

  public State[] parseStates() {
    State[] lStates;
    int lNumStates = mProperties.getIntValue("AutonomousNumStates", -1);
    System.out.println("Found " + Integer.toString(lNumStates) + " auto states");

    if (lNumStates < 1) {
      lStates = new State[1];
      lStates[0] = new WaitForTime(0);
      System.out.println("Insufficent number of states,  ");
    } else {
      lStates = new State[lNumStates];
      for (int i = 0; i < lNumStates; i++) {
        System.out.println("Parsing State " + Integer.toString(i + 1) + ",  ");

        String lStateName =
            mProperties.getStringValue("AutonomousState" + Integer.toString(i + 1), "");
        try {
          currentState = Class.forName(this.getClass().getPackage().getName() + "." + lStateName);
          // currentState = Class.forName( "missdaisy.autonomous." + lStateName);
          // there should only be one constructor
          mStateConstructor = currentState.getConstructors()[0];
          mNumberOfParameters = mStateConstructor.getParameterCount();

          Object mParamValues[] = new Object[mNumberOfParameters];

          mParamTypes = mStateConstructor.getParameterTypes();

          // fills parameter array with values
          System.out.println("Parsing parameters for state,  ");
          for (int k = 0; k < mStateConstructor.getParameters().length; k++) {
            // returns 0.0 or 0 if property cannot be found
            if (mParamTypes[k] == double.class) {
              mParamValues[k] = mProperties.getDoubleValue(
                  "AutonomousState" + Integer.toString(i + 1) + "Param" + Integer.toString(k + 1),
                  0.0);
            } else if (mParamTypes[k] == int.class) {
              // truncates value from autonomous file to be an
              // integer (if a number is entered as a double,
              // but is meant to be an integer)
              mParamValues[k] = (int) mProperties.getDoubleValue(
                  "AutonomousState" + Integer.toString(i + 1) + "Param" + Integer.toString(k + 1),
                  0.0);
            } else if (mParamTypes[k] == String.class) {
              mParamValues[k] = mProperties.getStringValue(
                  "AutonomousState" + Integer.toString(i + 1) + "Param" + Integer.toString(k + 1),
                  null);
            }
          }

          try {
            lStates[i] = (State) mStateConstructor.newInstance(mParamValues);
            System.out.println("Instantiated " + lStates[i].toString() + " with parameter(s): "
                + formatParam(mParamValues));
          } catch (InstantiationException | IllegalAccessException | InvocationTargetException
              | IllegalArgumentException e) {
            e.printStackTrace();
            System.out.println("Error instantiating " + "AutonomousState" + Integer.toString(i + 1)
                + ": " + lStateName + ". WaitForTime instantiated instead.");
          }

        } catch (ClassNotFoundException | SecurityException e) {
          System.out.println("Could not find specified state: " + lStateName
              + ". WaitForTime (0) instantiated instead.");
        }

        if (lStates[i] == null) {
          lStates[i] = new WaitForTime(0);
        }
      }
    }
    return lStates;
  }

  private String formatParam(Object[] paramValues) {
    String lFormattedString = "";
    for (int k = 0; k < paramValues.length; k++) {
      lFormattedString += paramValues[k].toString();
      if (k == paramValues.length - 1) {
        break;
      }

      lFormattedString += ", ";
    }
    return lFormattedString;
  }
}
