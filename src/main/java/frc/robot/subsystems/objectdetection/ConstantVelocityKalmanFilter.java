package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;

public class ConstantVelocityKalmanFilter {
  // State: [x, vx, y, vy]
  private Matrix<N4, N1> state;

  // State covariance matrix
  private Matrix<N4, N4> P;

  // Process noise covariance
  private Matrix<N4, N4> Q;

  // Measurement noise covariance
  private Matrix<N2, N2> R;

  // Measurement matrix (we only measure position, not velocity)
  private Matrix<N2, N4> H;

  // State transition matrix (will be updated with dt each step)
  private Matrix<N4, N4> F;

  private double lastTimestamp;
  private boolean initialized = false;

  public ConstantVelocityKalmanFilter(
      double processNoisePos, double processNoiseVel, double measurementNoise) {

    // Initialize state vector
    state = new Matrix<>(Nat.N4(), Nat.N1());

    // Initialize covariance matrix with high uncertainty
    P = Matrix.eye(Nat.N4());
    P.set(0, 0, 1.0); // x uncertainty
    P.set(1, 1, 10.0); // vx uncertainty (high initial uncertainty)
    P.set(2, 2, 1.0); // y uncertainty
    P.set(3, 3, 10.0); // vy uncertainty

    // Process noise covariance
    Q = new Matrix<>(Nat.N4(), Nat.N4());
    Q.set(0, 0, processNoisePos);
    Q.set(1, 1, processNoiseVel);
    Q.set(2, 2, processNoisePos);
    Q.set(3, 3, processNoiseVel);

    // Measurement noise covariance (only position measurements)
    R = Matrix.eye(Nat.N2()).times(measurementNoise);

    // Measurement matrix - we only observe position [x, y]
    H = new Matrix<>(Nat.N2(), Nat.N4());
    H.set(0, 0, 1.0); // Measure x
    H.set(1, 2, 1.0); // Measure y

    // State transition matrix (will be filled in predict())
    F = Matrix.eye(Nat.N4());
  }

  public void update(Pose2d measurement, double timestamp) {
    if (!initialized) {
      // Initialize state with first measurement
      state.set(0, 0, measurement.getX());
      state.set(1, 0, 0.0); // Initial velocity = 0
      state.set(2, 0, measurement.getY());
      state.set(3, 0, 0.0); // Initial velocity = 0

      lastTimestamp = timestamp;
      initialized = true;
      return;
    }

    double dt = timestamp - lastTimestamp;
    if (dt <= 0) {
      dt = 0.02; // Default to 20ms if timestamp issues
    }

    // Prediction step
    predict(dt);

    // Update step with measurement
    correct(measurement);

    lastTimestamp = timestamp;
  }

  public void predict(double dt) {
    // Update state transition matrix with dt
    // F = [1  dt  0  0 ]
    //     [0  1   0  0 ]
    //     [0  0   1  dt]
    //     [0  0   0  1 ]
    F = Matrix.eye(Nat.N4());
    F.set(0, 1, dt); // x = x + vx*dt
    F.set(2, 3, dt); // y = y + vy*dt

    // Predict state: x_pred = F * x
    state = F.times(state);

    // Predict covariance: P_pred = F * P * F^T + Q
    P = F.times(P).times(F.transpose()).plus(Q);
  }

  public void predictToTimestamp(double timestamp) {
    if (!initialized) {
      return;
    }

    double dt = timestamp - lastTimestamp;
    if (dt > 0) {
      predict(dt);
      lastTimestamp = timestamp;
    }
  }

  private void correct(Pose2d measurement) {
    // Measurement vector z = [x, y]
    Matrix<N2, N1> z = VecBuilder.fill(measurement.getX(), measurement.getY());

    // Innovation: y = z - H * x_pred
    Matrix<N2, N1> innovation = z.minus(H.times(state));

    // Innovation covariance: S = H * P * H^T + R
    Matrix<N2, N2> S = H.times(P).times(H.transpose()).plus(R);

    // Kalman gain: K = P * H^T * S^-1
    Matrix<N4, N2> K = P.times(H.transpose()).times(S.inv());

    // Update state: x = x_pred + K * y
    state = state.plus(K.times(innovation));

    // Update covariance: P = (I - K * H) * P
    Matrix<N4, N4> I_KH = Matrix.eye(Nat.N4()).minus(K.times(H));
    P = I_KH.times(P);
  }

  public Pose2d getEstimatedPose() {
    return new Pose2d(new Translation2d(state.get(0, 0), state.get(2, 0)), new Rotation2d());
  }

  public Translation2d getEstimatedVelocity() {
    return new Translation2d(state.get(1, 0), state.get(3, 0));
  }

  public double getSpeed() {
    double vx = state.get(1, 0);
    double vy = state.get(3, 0);
    return Math.sqrt(vx * vx + vy * vy);
  }
}
