package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;

public class FieldStates {
  private static boolean[] algaeLocations = new boolean[] {true, true, true, true, true, true};

  private static final TunableNumberGroup tunableGroup = new TunableNumberGroup("FieldStates");
  private static final LoggedTunableNumber tunableX = tunableGroup.build("TunableX", 3.758);
  private static final LoggedTunableNumber tunableY = tunableGroup.build("TunableY", 4.19);
  private static final LoggedTunableNumber tunableZ = tunableGroup.build("TunableZ", 1.108);
  private static final LoggedTunableNumber tunableYaw = tunableGroup.build("TunableYaw", 0);
  private static final LoggedTunableNumber tunablePitch = tunableGroup.build("TunablePitch", 35);
  private static final LoggedTunableNumber tunableRoll = tunableGroup.build("TunableRoll", 0);

  private static final LoggerGroup logGroup = LoggerGroup.build("FieldStates");
  private static final LoggerEntry.Struct<Pose3d> testPose =
      logGroup.buildStruct(Pose3d.class, "TestPose");
  private static final LoggerEntry.StructArray<Pose3d> algaePoses =
      logGroup.buildStructArray(Pose3d.class, "AlgaePoses");

  private static final Pose3d blueNearMidAlgae = new Pose3d(3.812, 4.0265, 0.905, Rotation3d.kZero);
  private static final double upperAlgaeHeight = 1.308;

  List<Pose3d> algaePoseArray = new ArrayList<>();

  {
    for (int i = 0; i < algaeLocations.length; i++) {
      if (algaeLocations[i]) {
        Pose3d referencePose = blueNearMidAlgae;
        Pose3d correctedHeight =
            new Pose3d(
                referencePose.getX(),
                referencePose.getY(),
                i % 2 == 1 ? upperAlgaeHeight : referencePose.getZ(),
                referencePose.getRotation());
        algaePoseArray.add(
            AllianceFlipUtil.flipPoseForAlliance(
                GeometryUtil.rotatePose3dAroundTranslation2d(
                    correctedHeight,
                    Constants.FieldConstants.BLUE_HUB_CENTER_POSE,
                    Rotation2d.fromDegrees(60 * (i - 1)))));
      }
    }

    testPose.info(
        new Pose3d(
            tunableX.get(),
            tunableY.get(),
            tunableZ.get(),
            new Rotation3d(
                Math.toRadians(tunableRoll.get()),
                Math.toRadians(tunablePitch.get()),
                Math.toRadians(tunableYaw.get()))));

    algaePoses.info(algaePoseArray.toArray(new Pose3d[algaePoseArray.size()]));
  }
}
