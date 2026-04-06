package subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.objectdetection.ObjectDetectionAlgorithms;
import frc.robot.subsystems.objectdetection.ObjectDetectionAlgorithms.TimestampedFieldDetection;
import java.util.List;
import org.junit.jupiter.api.Test;

class ObjectDetectionAlgorithmsTest {

  private TimestampedFieldDetection pt(double x, double y) {
    return new TimestampedFieldDetection(x, y, 0.0);
  }

  @Test
  void dbscan_twoSeparateClusters_returnsTwoClusters() {
    List<TimestampedFieldDetection> points =
        List.of(
            pt(0, 0), pt(0.1, 0), pt(0.2, 0), // cluster A
            pt(5, 5), pt(5.1, 5), pt(5.2, 5)); // cluster B

    List<List<TimestampedFieldDetection>> clusters =
        ObjectDetectionAlgorithms.dbscan(points, 0.5, 2);

    assertEquals(2, clusters.size());
    assertEquals(3, clusters.get(0).size());
    assertEquals(3, clusters.get(1).size());
  }

  @Test
  void dbscan_isolatedNoisyPoint_isDiscarded() {
    List<TimestampedFieldDetection> points =
        List.of(
            pt(0, 0), pt(0.1, 0), pt(0.2, 0), // cluster
            pt(10, 10)); // isolated noise

    List<List<TimestampedFieldDetection>> clusters =
        ObjectDetectionAlgorithms.dbscan(points, 0.5, 2);

    assertEquals(1, clusters.size());
    assertEquals(3, clusters.get(0).size());
  }

  @Test
  void dbscan_emptyInput_returnsEmptyList() {
    assertTrue(ObjectDetectionAlgorithms.dbscan(List.of(), 0.5, 2).isEmpty());
  }

  @Test
  void dbscan_allPointsTooFarApart_returnsNoClusters() {
    List<TimestampedFieldDetection> points = List.of(pt(0, 0), pt(2, 0), pt(4, 0));
    assertTrue(ObjectDetectionAlgorithms.dbscan(points, 0.5, 2).isEmpty());
  }

  // ---- scoring tests ----

  @Test
  void scoreCluster_closerClusterScoresHigher() {
    List<TimestampedFieldDetection> close =
        List.of(
            new TimestampedFieldDetection(1, 0, 0),
            new TimestampedFieldDetection(1.1, 0, 0));
    List<TimestampedFieldDetection> far =
        List.of(
            new TimestampedFieldDetection(10, 0, 0),
            new TimestampedFieldDetection(10.1, 0, 0));

    Translation2d robot = new Translation2d(0, 0);
    double scoreClose = ObjectDetectionAlgorithms.scoreCluster(close, robot, 0, 2.0, 1.0, 0.5);
    double scoreFar = ObjectDetectionAlgorithms.scoreCluster(far, robot, 0, 2.0, 1.0, 0.5);

    assertTrue(scoreClose > scoreFar, "Closer cluster should score higher");
  }

  @Test
  void scoreCluster_largerClusterScoresHigher() {
    List<TimestampedFieldDetection> large =
        List.of(
            new TimestampedFieldDetection(5, 0, 0),
            new TimestampedFieldDetection(5.1, 0, 0),
            new TimestampedFieldDetection(5.2, 0, 0));
    List<TimestampedFieldDetection> small =
        List.of(
            new TimestampedFieldDetection(5, 0, 0),
            new TimestampedFieldDetection(5.1, 0, 0));

    Translation2d robot = new Translation2d(0, 0);
    assertTrue(
        ObjectDetectionAlgorithms.scoreCluster(large, robot, 0, 2.0, 1.0, 0.5)
            > ObjectDetectionAlgorithms.scoreCluster(small, robot, 0, 2.0, 1.0, 0.5));
  }

  @Test
  void scoreCluster_olderPointsReduceScore() {
    List<TimestampedFieldDetection> fresh =
        List.of(
            new TimestampedFieldDetection(5, 0, 1.0),
            new TimestampedFieldDetection(5.1, 0, 1.0));
    List<TimestampedFieldDetection> stale =
        List.of(
            new TimestampedFieldDetection(5, 0, 0.0),
            new TimestampedFieldDetection(5.1, 0, 0.0));

    Translation2d robot = new Translation2d(0, 0);
    double now = 1.0;
    assertTrue(
        ObjectDetectionAlgorithms.scoreCluster(fresh, robot, now, 2.0, 1.0, 0.5)
            > ObjectDetectionAlgorithms.scoreCluster(stale, robot, now, 2.0, 1.0, 0.5));
  }

  // ---- PCA tests ----

  @Test
  void fitPrincipalAxis_horizontalLine_returnsHorizontalAxis() {
    List<TimestampedFieldDetection> points =
        List.of(
            new TimestampedFieldDetection(-1, 0, 0),
            new TimestampedFieldDetection(0, 0, 0),
            new TimestampedFieldDetection(1, 0, 0));

    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(points);

    assertEquals(0.0, axis.centerX(), 1e-9);
    assertEquals(0.0, axis.centerY(), 1e-9);
    assertEquals(1.0, Math.abs(axis.axisX()), 1e-9);
    assertEquals(0.0, Math.abs(axis.axisY()), 1e-9);
    assertEquals(1.0, axis.semiMajor(), 1e-9);
  }

  @Test
  void fitPrincipalAxis_verticalLine_returnsVerticalAxis() {
    List<TimestampedFieldDetection> points =
        List.of(
            new TimestampedFieldDetection(0, -2, 0),
            new TimestampedFieldDetection(0, 0, 0),
            new TimestampedFieldDetection(0, 2, 0));

    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(points);

    assertEquals(0.0, axis.centerX(), 1e-9);
    assertEquals(0.0, axis.centerY(), 1e-9);
    assertEquals(0.0, Math.abs(axis.axisX()), 1e-9);
    assertEquals(1.0, Math.abs(axis.axisY()), 1e-9);
    assertEquals(2.0, axis.semiMajor(), 1e-9);
  }

  @Test
  void fitPrincipalAxis_twoPoints_halfDistanceIsSemiMajor() {
    List<TimestampedFieldDetection> points =
        List.of(
            new TimestampedFieldDetection(0, 0, 0),
            new TimestampedFieldDetection(3, 4, 0));

    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(points);

    assertEquals(1.5, axis.centerX(), 1e-9);
    assertEquals(2.0, axis.centerY(), 1e-9);
    assertEquals(2.5, axis.semiMajor(), 1e-9);
  }

  // ---- waypoint generation tests ----

  @Test
  void generateCollectionWaypoints_robotOnLeft_firstWaypointIsLeftmost() {
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Translation2d robot = new Translation2d(-3, 0);

    Pose2d[] wps = ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, robot);

    assertEquals(-1.0, wps[0].getX(), 1e-9, "First waypoint should be leftmost (x=-1)");
    assertEquals(1.0, wps[3].getX(), 1e-9, "Last waypoint should be rightmost (x=+1)");
  }

  @Test
  void generateCollectionWaypoints_robotOnRight_firstWaypointIsRightmost() {
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Translation2d robot = new Translation2d(3, 0);

    Pose2d[] wps = ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, robot);

    assertEquals(1.0, wps[0].getX(), 1e-9, "First waypoint should be rightmost (x=+1)");
    assertEquals(-1.0, wps[3].getX(), 1e-9, "Last waypoint should be leftmost (x=-1)");
  }

  @Test
  void generateCollectionWaypoints_returns4Waypoints() {
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Pose2d[] wps =
        ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, new Translation2d(0, 0));
    assertEquals(4, wps.length);
  }

  @Test
  void generateCollectionWaypoints_travelDirectionPointsAlongAxis() {
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Pose2d[] wps =
        ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, new Translation2d(-3, 0));

    for (int i = 0; i < 3; i++) {
      assertEquals(
          0.0,
          wps[i].getRotation().getDegrees(),
          1.0,
          "Waypoint " + i + " travel direction should be ~0°");
    }
  }
}
