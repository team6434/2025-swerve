package frc.robot;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class ThriftyEncoder {

  public AnalogEncoder encoder;

  public ThriftyEncoder(int analogChannel) {
    encoder = new AnalogEncoder(analogChannel);
  } 

  public void setDistancePerRotation(double config) {
    encoder.setDistancePerRotation(config);
  }

  public double rawAbsPosition() { // absPos. without offset
    return encoder.getAbsolutePosition();
  }

  public double absPosition(double offsetValue) { // absPos. with offset
    return encoder.getAbsolutePosition() - offsetValue;
  }

  public double getDistance() {
    return encoder.getDistance();
  }

  // public void setOffset(double offsetValue) {
  //   encoder.setPositionOffset(offsetValue);
  // }
}