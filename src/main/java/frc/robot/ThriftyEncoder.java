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

  public double rawAbsPosition() {
    return encoder.getAbsolutePosition();
  }

  public double absPosition(double offsetValue) {
    return encoder.getAbsolutePosition() - offsetValue;
  }

  // public void setOffset(double offsetValue) {
  //   encoder.setPositionOffset(offsetValue);
  // }
}