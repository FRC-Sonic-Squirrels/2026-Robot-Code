package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class CameraConfiguration {
  public Distance x;
  public Distance y;
  public Distance z;
  public Angle    roll;
  public Angle    pitch;
  public Angle    yaw;

  public void rotate180() {
    x   = x.times(-1);
    y   = y.times(-1);
    yaw = yaw.plus(Units.Degree.of(180));
  }

  public Transform3d toTransform3d() {
    var pos = new Translation3d(x.in(Units.Meters), y.in(Units.Meters), z.in(Units.Meters));
    var rot = new Rotation3d(roll.in(Units.Radians), pitch.in(Units.Radians), yaw.in(Units.Radians));
    return new Transform3d(pos, rot);
  }
}
