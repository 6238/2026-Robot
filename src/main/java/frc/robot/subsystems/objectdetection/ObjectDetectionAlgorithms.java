package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

class ObjectDetectionAlgorithms {

  record TimestampedFieldDetection(double x, double y, double timestamp) {}

  record PrincipalAxis(
      double centerX, double centerY, double axisX, double axisY, double semiMajor) {}

  // -------------------------------------------------------------------------
  // DBSCAN
  // -------------------------------------------------------------------------

  static List<List<TimestampedFieldDetection>> dbscan(
      List<TimestampedFieldDetection> points, double epsilon, int minPts) {
    int n = points.size();
    int[] label = new int[n]; // 0=unvisited, -1=noise, >0=cluster id
    List<List<TimestampedFieldDetection>> clusters = new ArrayList<>();
    int clusterCount = 0;

    for (int i = 0; i < n; i++) {
      if (label[i] != 0) continue;

      List<Integer> neighbors = regionQuery(points, i, epsilon);
      if (neighbors.size() < minPts) {
        label[i] = -1;
        continue;
      }

      clusterCount++;
      List<TimestampedFieldDetection> cluster = new ArrayList<>();
      clusters.add(cluster);
      label[i] = clusterCount;
      cluster.add(points.get(i));

      ArrayDeque<Integer> queue = new ArrayDeque<>(neighbors);
      while (!queue.isEmpty()) {
        int q = queue.poll();
        if (label[q] == -1) label[q] = clusterCount; // border point
        if (label[q] != 0) continue;

        label[q] = clusterCount;
        cluster.add(points.get(q));

        List<Integer> qNeighbors = regionQuery(points, q, epsilon);
        if (qNeighbors.size() >= minPts) queue.addAll(qNeighbors);
      }
    }

    return clusters;
  }

  private static List<Integer> regionQuery(
      List<TimestampedFieldDetection> points, int idx, double epsilon) {
    List<Integer> neighbors = new ArrayList<>();
    double px = points.get(idx).x();
    double py = points.get(idx).y();
    double eps2 = epsilon * epsilon;
    for (int i = 0; i < points.size(); i++) {
      double dx = points.get(i).x() - px;
      double dy = points.get(i).y() - py;
      if (dx * dx + dy * dy <= eps2) neighbors.add(i);
    }
    return neighbors;
  }

  // -------------------------------------------------------------------------
  // Cluster scoring
  // -------------------------------------------------------------------------

  static double scoreCluster(
      List<TimestampedFieldDetection> cluster,
      Translation2d robotPos,
      double now,
      double decayK,
      double countScalar,
      double distScalar) {
    double weightedCount = 0;
    double sumX = 0, sumY = 0;
    for (TimestampedFieldDetection p : cluster) {
      double age = now - p.timestamp();
      weightedCount += Math.exp(-decayK * age);
      sumX += p.x();
      sumY += p.y();
    }
    double cx = sumX / cluster.size();
    double cy = sumY / cluster.size();
    double dx = cx - robotPos.getX();
    double dy = cy - robotPos.getY();
    return weightedCount * countScalar - (dx * dx + dy * dy) * distScalar;
  }

  static Translation2d clusterCentroid(List<TimestampedFieldDetection> cluster) {
    double sumX = 0, sumY = 0;
    for (TimestampedFieldDetection p : cluster) {
      sumX += p.x();
      sumY += p.y();
    }
    return new Translation2d(sumX / cluster.size(), sumY / cluster.size());
  }

  // -------------------------------------------------------------------------
  // PCA principal axis fitting
  // -------------------------------------------------------------------------

  static PrincipalAxis fitPrincipalAxis(List<TimestampedFieldDetection> cluster) {
    int n = cluster.size();
    double cx = 0, cy = 0;
    for (TimestampedFieldDetection p : cluster) {
      cx += p.x();
      cy += p.y();
    }
    cx /= n;
    cy /= n;

    double sxx = 0, sxy = 0, syy = 0;
    for (TimestampedFieldDetection p : cluster) {
      double dx = p.x() - cx;
      double dy = p.y() - cy;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }
    sxx /= n;
    sxy /= n;
    syy /= n;

    double disc = Math.sqrt(Math.max(0, (sxx - syy) * (sxx - syy) / 4.0 + sxy * sxy));
    double lambda1 = (sxx + syy) / 2.0 + disc;

    double vx, vy;
    if (Math.abs(sxy) > 1e-10) {
      vx = lambda1 - syy;
      vy = sxy;
    } else {
      vx = (sxx >= syy) ? 1.0 : 0.0;
      vy = (sxx >= syy) ? 0.0 : 1.0;
    }
    double norm = Math.hypot(vx, vy);
    if (norm < 1e-10) {
      vx = 1.0;
      vy = 0.0;
    } else {
      vx /= norm;
      vy /= norm;
    }

    double minProj = Double.MAX_VALUE, maxProj = -Double.MAX_VALUE;
    for (TimestampedFieldDetection p : cluster) {
      double proj = (p.x() - cx) * vx + (p.y() - cy) * vy;
      minProj = Math.min(minProj, proj);
      maxProj = Math.max(maxProj, proj);
    }

    double midProj = (minProj + maxProj) / 2.0;
    return new PrincipalAxis(
        cx + midProj * vx, cy + midProj * vy, vx, vy, (maxProj - minProj) / 2.0);
  }

  // -------------------------------------------------------------------------
  // 4-waypoint generation
  // -------------------------------------------------------------------------

  /**
   * Generates 4 Pose2d collection waypoints along the principal axis. Pose rotation = direction of
   * travel (path tangent for PathPlanner.waypointsFromPoses — NOT chassis heading). The first
   * waypoint is the one closest to robotPos.
   */
  static Pose2d[] generateCollectionWaypoints(PrincipalAxis axis, Translation2d robotPos) {
    double a = axis.semiMajor();
    double[] offsets = {-a, -a / 3.0, a / 3.0, a};

    Translation2d[] positions = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      positions[i] =
          new Translation2d(
              axis.centerX() + offsets[i] * axis.axisX(),
              axis.centerY() + offsets[i] * axis.axisY());
    }

    if (positions[3].getDistance(robotPos) < positions[0].getDistance(robotPos)) {
      Translation2d[] reversed = new Translation2d[4];
      for (int i = 0; i < 4; i++) reversed[i] = positions[3 - i];
      positions = reversed;
    }

    Pose2d[] waypoints = new Pose2d[4];
    for (int i = 0; i < 3; i++) {
      Translation2d delta = positions[i + 1].minus(positions[i]);
      waypoints[i] = new Pose2d(positions[i], new Rotation2d(delta.getX(), delta.getY()));
    }
    waypoints[3] = new Pose2d(positions[3], waypoints[2].getRotation());

    return waypoints;
  }
}
