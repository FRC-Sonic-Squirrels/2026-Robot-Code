package frc.robot.visualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SimpleMechanismVisualization {
  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Mechanism simpleMechanism;

  }

  public static class Tunables extends AutoRegisterTunables {
    public LoggedTunableNumber testTunable;
  }

  private final Logs     logs     = LoggerGroup.root.newAutoLogger(Logs.class, "SimpleMechanismVisualization");
  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, "SimpleMechanismVisualization");

  LoggedMechanism2d mechanism2d = new LoggedMechanism2d(Units.Inches.of(32.0).in(Units.Meters),
      Units.Inches.of(60.0).in(Units.Meters));

  LoggedMechanismRoot2d mechRoot = mechanism2d.getRoot("mechRoot", Units.Inches.of(9.75).in(Units.Meters),
      Units.Inches.of(4).in(Units.Meters));

  LoggedMechanismLigament2d elevatorLigament = mechRoot.append(new LoggedMechanismLigament2d("mech", 0.001, 90));

  LoggedMechanismLigament2d armLigament = elevatorLigament
      .append(new LoggedMechanismLigament2d("mech", ArmConstants.ARM_LENGTH.in(Units.Meters), 90));

  public SimpleMechanismVisualization() {
    elevatorLigament.setColor(new Color8Bit(0, 0, 255));
    elevatorLigament.setLineWeight(15.0);
    armLigament.setLineWeight(15.0);
  }

  public void updateVisualization(Distance elevatorHeight, Rotation2d armAngle) {
    if (elevatorHeight.in(Units.Inch) != 0.0)
      elevatorLigament.setLength(elevatorHeight.in(Units.Meters));
    armLigament.setAngle(armAngle.minus(Rotation2d.kCCW_90deg));
  }

  public void logMechanism() {
    logs.simpleMechanism.info(mechanism2d);
  }
}
