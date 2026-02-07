package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class GeometryUtilTest {

  @Test
  void isRotationAtTarget_exactMatch_isTrue() {
    var angle  = Rotation2d.fromDegrees(42.0);
    var target = Rotation2d.fromDegrees(42.0);

    assertTrue(GeometryUtil.isRotationAtTarget(angle, target, 0.0));
  }

  @Test
  void isRotationAtTarget_onToleranceBoundary_isTrue() {
    var angle  = Rotation2d.fromDegrees(10.0);
    var target = Rotation2d.fromDegrees(20.0);

    // error = -10 deg, abs(error) == tolerance -> inclusive
    assertTrue(GeometryUtil.isRotationAtTarget(angle, target, 10.0));
  }

  @Test
  void isRotationAtTarget_justOutsideTolerance_isFalse() {
    var angle  = Rotation2d.fromDegrees(10.0);
    var target = Rotation2d.fromDegrees(20.0);

    assertFalse(GeometryUtil.isRotationAtTarget(angle, target, 9.999));
  }

  @Test
  void isRotationAtTarget_wrapAroundAcross180Degrees_usesShortestPath() {
    // -179 and +179 are only 2 degrees apart when normalized
    var angle  = Rotation2d.fromDegrees(-179.0);
    var target = Rotation2d.fromDegrees(179.0);

    assertTrue(GeometryUtil.isRotationAtTarget(angle, target, 2.0));
    assertFalse(GeometryUtil.isRotationAtTarget(angle, target, 1.999));
  }

  @Test
  void isRotationAtTarget_rotationToleranceOverload_matchesDoubleOverload() {
    var angle     = Rotation2d.fromDegrees(15.0);
    var target    = Rotation2d.fromDegrees(20.0);
    var tolerance = Rotation2d.fromDegrees(5.0);

    assertEquals(GeometryUtil.isRotationAtTarget(angle, target, tolerance.getDegrees()),
        GeometryUtil.isRotationAtTarget(angle, target, tolerance));
  }
}
