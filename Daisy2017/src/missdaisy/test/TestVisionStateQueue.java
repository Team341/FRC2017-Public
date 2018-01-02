package missdaisy.test;

public class TestVisionStateQueue {
/*
  public static class VisionState {

    public double receivedTime;
    public Pixy.Block pkt;
    public Navigation.State navState;
    public double turretYaw;
    public double shooterRPM;

    public VisionState(double time, Pixy.Block pkt, Navigation.State navState, double turretYaw,
        double shooterRPM) {
      this.receivedTime = time;
      this.pkt = pkt;
      this.navState = navState;
      this.turretYaw = turretYaw;
      this.shooterRPM = shooterRPM;
    }
  }

  public static class PixyThread {

    public static PixyThread mInstance = null;
    public ConcurrentSkipListMap<Double, VisionState> history = new ConcurrentSkipListMap<Double, VisionState>();

    public static PixyThread getInstance() {
      if (mInstance == null) {
        mInstance = new PixyThread();
      }
      return mInstance;
    }

    public void Add(double currTime, VisionState currentState) {
      // Put the new state into the history queue
      history.put(currTime, currentState);

      // Prune old state blocks from the history queue
      ConcurrentNavigableMap<Double, VisionState> removeMap = history.headMap(currTime - 1000);
      NavigableSet navigableRemoveMapKeySet = removeMap.keySet();
      for (Iterator iterator = navigableRemoveMapKeySet.iterator(); iterator.hasNext(); ) {
        history.remove(iterator.next());
      }
    }


    public VisionState getStateAtTime(double time) {
      // Start at the beginning of the queue and search backwards for the
      // closest time
      VisionState returnState = null;

      Entry<Double, VisionState> lowerBound = history.floorEntry(time);
      Entry<Double, VisionState> upperBound = history.ceilingEntry(time);

      if (lowerBound == null && upperBound == null) {
        // No data in the queue?

      } else if (lowerBound == null) {
        // We only have the upper bound, so propagate forward
        double dt = time - upperBound.getKey();
        returnState = PropagateTo(upperBound.getValue(), dt);

      } else if (upperBound == null) {
        // We only have the lower bound, so back propagate
        double dt = time - lowerBound.getKey();
        returnState = PropagateTo(lowerBound.getValue(), dt);
      } else {
        // Interpolate between the two values
        returnState = Interpolate(lowerBound.getValue(), upperBound.getValue(), time);
      }

      return returnState;
    }

    public VisionState Interpolate(VisionState lower, VisionState upper, double time) {

      if (lower == upper) {
        return lower;
      }

      double ratio = (time - lower.receivedTime) / (upper.receivedTime - lower.receivedTime);

      if (ratio == 0.0) {
        return lower;
      } else if (ratio == 1.0) {
        return upper;
      }

      Pixy.Block pkt = lower.pkt;
      if ((time - lower.receivedTime) < (upper.receivedTime - time)) {
        pkt = upper.pkt;
      }

      double dX = upper.navState.x - lower.navState.x;
      double dY = upper.navState.y - lower.navState.y;
      double dYaw = upper.navState.heading - lower.navState.heading;
      double dVel = upper.navState.velocity - lower.navState.velocity;
      double dYawRate = upper.navState.headingRate - lower.navState.headingRate;

      Navigation.State newNavState = new Navigation.State(lower.navState.x + ratio * dX,
          lower.navState.y + ratio * dY,
          DaisyMath.boundAngleNeg180to180Degrees(lower.navState.heading + ratio * dYaw),
          lower.navState.velocity + ratio * dVel,
          DaisyMath.boundAngleNeg180to180Degrees(lower.navState.headingRate + ratio * dYawRate));

      double dTurretYaw = upper.turretYaw - lower.turretYaw;
      double turretYaw = lower.turretYaw + ratio * dTurretYaw;

      double dRPM = upper.shooterRPM - lower.shooterRPM;
      double shooterRPM = lower.shooterRPM + ratio * dRPM;

      VisionState newState = new VisionState(time, pkt, newNavState, turretYaw, shooterRPM);

      return newState;
    }

    public VisionState PropagateTo(VisionState currState, double dt) {

      // Calculate the time we're propagating to
      double newTime = currState.receivedTime + dt;

      // Just copy the PixyBlock data for now
      Pixy.Block pkt = currState.pkt;

      // Propagate the robot drivebase to the desired time
      Navigation.State newNavState = currState.navState;
      newNavState.heading += dt * currState.navState.headingRate;
      newNavState.x += dt * currState.navState.velocity * Math
          .cos(newNavState.heading * Constants.UnitConv.DEG_TO_RAD);
      newNavState.y += dt * currState.navState.velocity * Math
          .sin(newNavState.heading * Constants.UnitConv.DEG_TO_RAD);

      // Propagate the turret angle
      double turretYaw = currState.turretYaw;

      // Hold the shooter rpm current for now
      double shooterRPM = currState.shooterRPM;

      VisionState newState = new VisionState(newTime, pkt, newNavState, turretYaw, shooterRPM);

      return newState;
    }

  }

  public static void main(String[] args) {

    //ConcurrentSkipListMap<Double, VisionState> history = new ConcurrentSkipListMap<Double, VisionState>();
    double x = 0;
    double y = 0;
    double vel = 10;
    double heading = 0;
    for (int t = 0; t < 10; t++) {

      // Estimate what the target will look like
      PixyBlock newPkt = new PixyBlock();
      newPkt.X = 10;
      newPkt.Y = 10;
      newPkt.Height = 20;
      newPkt.Width = 60;

      // Propagate the robot forward
      double evalTime = (double) t;
      double dt = 1.0;
      heading = 1.0 / 10.0 * Math.PI * Math.sin(Math.PI * evalTime / 10.0);
      double headingRate =
          Math.pow(1.0 / 10.0 * Math.PI, 2.0) * Math.cos(Math.PI * evalTime / 10.0);
      x += dt * vel * Math.cos(heading);
      y += dt * vel * Math.sin(heading);

      Navigation.State newState = new Navigation.State(x, y, heading, vel, headingRate);

      double turretYaw = heading;

      double shooterRPM = 3500;

      VisionState newVisionState = new VisionState((double) t, newPkt, newState, turretYaw,
          shooterRPM);

    }
  }*/
}
