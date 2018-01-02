package missdaisy.tracking;

import missdaisy.utilities.DaisyMath;

public class ReferenceFrame {
  double positionX;
  double positionY;
  double yaw;
  double cosAng;
  double sinAng;

  public class Coordinates {
    double x;
    double y;
  }

  public ReferenceFrame(double x, double y, double angle) {
    setX(x);
    setY(y);
    setAngle(angle);
  }

  public ReferenceFrame(ReferenceFrame frame) {
    this(frame.positionX, frame.positionY, frame.yaw);
  }

  public void setX(double x) {
    this.positionX = x;
  }

  public void setY(double y) {
    this.positionY = y;
  }

  public void setAngle(double angle) {
    this.yaw = angle;
    this.cosAng = Math.cos(angle);
    this.sinAng = Math.sin(angle);
  }

  public double getX() {
    return positionX;
  }

  public double getY() {
    return positionY;
  }

  public double getYaw() {
    return yaw;
  }

  public double getCosAng() {
    return cosAng;
  }

  public double getSinAng() {
    return sinAng;
  }

  public ReferenceFrame rotateFrom(ReferenceFrame other) {
    // This rotates the "other" frame from "this" frame
    double newX = other.positionX * this.cosAng - other.positionY * this.sinAng + this.positionX;
    double newY = other.positionX * this.sinAng + other.positionY * this.cosAng + this.positionY;
    double newAng = DaisyMath.boundAngleNegPiToPiRadians(other.yaw - this.yaw);

    return new ReferenceFrame(newX, newY, newAng);
  }

  public ReferenceFrame mapTo(ReferenceFrame other) {
    // This rotates the "other" frame from "this" frame
    double newX = this.positionX * other.cosAng - this.positionY * other.sinAng + other.positionX;
    double newY = this.positionX * other.sinAng + this.positionY * other.cosAng + other.positionY;
    double newAng = DaisyMath.boundAngleNegPiToPiRadians(other.yaw + this.yaw);

    return new ReferenceFrame(newX, newY, newAng);
  }

  public ReferenceFrame rotateTo(ReferenceFrame other) {
    // This rotates the "other" frame to "this" frame
    double x = other.positionX - this.positionX;
    double y = other.positionY - this.positionY;
    double newX = x * this.cosAng + y * this.sinAng;
    double newY = -x * this.sinAng + y * this.cosAng;
    double newAng = DaisyMath.boundAngleNegPiToPiRadians(other.yaw - this.yaw);

    return new ReferenceFrame(newX, newY, newAng);
  }

  public ReferenceFrame propagateFrame(double turn, double speed, double dt) {
    return new ReferenceFrame(this.getX() + dt * speed * this.getCosAng(),
        this.getY() + dt * speed * this.getSinAng(), this.getYaw() + dt * turn);
  }

  public double findDistance(ReferenceFrame other) {
    return Math.hypot(other.positionX - this.positionX, other.positionY - this.positionY);
  }

  public double findAngle(ReferenceFrame other) {
    return Math.atan2(other.positionY - this.positionY, other.positionX - this.positionX);
  }

  public ReferenceFrame interp(ReferenceFrame other, double x) {
    return new ReferenceFrame(x * (other.positionX - positionX) + positionX,
        x * (other.positionY - positionY) + positionY, x * (other.yaw - yaw) + yaw);
  }
}
