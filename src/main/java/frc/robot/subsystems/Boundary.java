package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Boundary subsystem — 13x13 ft keepin zone with predictive wheel lock.
 *
 * Two-layer protection:
 *
 *  1. SLOWDOWN   — robot within SLOWDOWN_ZONE_M of a wall → translation speed
 *                  scaled down linearly to MIN_SCALAR.  getSpeedScalar() is
 *                  called from Drive before every drive command.
 *
 *  2. WHEEL LOCK — robot is at/past the wall AND the joystick input would push
 *                  it further outside → wheels lock immediately.
 *                  This stops odometry drift: without this, encoders keep
 *                  counting while the robot is physically stuck against the wall.
 *                  shouldLockWheels(inputX, inputY) is called from Drive before
 *                  any movement is applied.
 *
 * Prediction logic:
 *   predictedPos = currentPos + (inputDir * maxSpeed * PREDICT_DT)
 *   If that point is outside the boundary, the input is heading out → lock.
 *
 * NOTE — no circular init:
 *   Drive calls Boundary.getInstance() from driveCommand().
 *   Boundary MUST NOT call Drive.getInstance() in its constructor, or the two
 *   singletons will deadlock each other during construction.
 *   Drive is resolved lazily via getDrive() on the first actual method call,
 *   by which point Drive is fully constructed.
 */
public class Boundary extends SubsystemBase {

  // ── Tunable constants ────────────────────────────────────────────────────────

  /** Side length of the square boundary (13 ft in metres). */
  public static final double BOUNDARY_SIZE_M = 13.0 * 0.3048; // 3.9624 m

  /** Distance from a wall at which the speed scalar begins ramping down. */
  public static final double SLOWDOWN_ZONE_M = 0.75;

  /** Minimum speed scalar at the wall (15 % of max speed). */
  public static final double MIN_SCALAR = 0.15;

  /**
   * Look-ahead time (seconds) for the wheel-lock prediction.
   * 0.1 s at ~4 m/s max speed = ~0.4 m of look-ahead.
   * Increase to lock earlier; decrease to lock only when very close.
   */
  public static final double PREDICT_DT = 0.1;

  /**
   * Gate distance (metres): predictive lock is only considered when the robot
   * is already within this distance of a wall.  Prevents a full-speed joystick
   * flick in the middle of the field from triggering a false lock.
   */
  public static final double LOCK_GATE_M = 0.3;

  // ── Internal state ────────────────────────────────────────────────────────────

  /** Resolved lazily — DO NOT touch Drive in the constructor. */
  private Drive drive = null;

  /**
   * Boundary centre in field coordinates.
   * Set once on the first getDrive() call (i.e. first time a drive method runs),
   * which is after Drive is fully constructed.
   */
  private Translation2d centre = null;

  /** Half-side length for wall-distance math. */
  private final double half = BOUNDARY_SIZE_M / 2.0;

  /** Cached for SmartDashboard publishing in periodic(). */
  private boolean wheelsLocked = false;

  // ── Constructor ───────────────────────────────────────────────────────────────

  public Boundary() {
    // Intentionally empty — Drive must not be accessed here.
    // Drive() → Boundary.getInstance() → new Boundary() → Drive.getInstance()
    // would create an infinite construction loop.
    // getDrive() handles lazy init safely.
  }

  // ── Lazy Drive accessor ───────────────────────────────────────────────────────

  /**
   * Returns the Drive singleton, resolving it (and anchoring the boundary
   * centre) on the very first call.  Safe to call any time after Drive's
   * constructor has finished.
   */
  private Drive getDrive() {
    if (drive == null) {
      drive  = Drive.getInstance();
      centre = drive.getPose().getTranslation();
    }
    return drive;
  }

  // ── SubsystemBase ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Boundary/SpeedScalar",   getSpeedScalar());
    SmartDashboard.putNumber("Boundary/DistToWall_m",  getDistanceToNearestWall());
    SmartDashboard.putBoolean("Boundary/NearWall",     getDistanceToNearestWall() < SLOWDOWN_ZONE_M);
    SmartDashboard.putBoolean("Boundary/WheelsLocked", wheelsLocked);
  }

  // ── Public API ────────────────────────────────────────────────────────────────

  /**
   * PRIMARY METHOD — call this from Drive BEFORE applying any translation.
   *
   * Pass in the raw joystick X/Y values (post-deadband, field-relative, [-1,1]).
   * Returns true if the wheels should be locked this cycle.
   *
   * Lock fires when ALL of the following are true:
   *   a) Robot is within LOCK_GATE_M of a wall (gate check), AND
   *   b) Projecting the robot forward by PREDICT_DT at max speed puts it
   *      outside the boundary.
   *
   * @param fieldRelInputX  Joystick X after deadband, field-relative, [-1, 1]
   * @param fieldRelInputY  Joystick Y after deadband, field-relative, [-1, 1]
   * @return true = lock wheels; false = normal driving
   */
  public boolean shouldLockWheels(double fieldRelInputX, double fieldRelInputY) {

    // Gate: ignore prediction when well inside the boundary
    if (getDistanceToNearestWall() > LOCK_GATE_M) {
      wheelsLocked = false;
      return false;
    }

    // No input → no movement → no lock needed
    double inputMag = Math.hypot(fieldRelInputX, fieldRelInputY);
    if (inputMag < 1e-6) {
      wheelsLocked = false;
      return false;
    }

    // Predicted displacement:
    //   direction = (inputX, inputY) normalised
    //   magnitude = inputMag * maxSpeed (joystick [-1,1] maps to [0, maxSpeed])
    //   displacement = direction * magnitude * dt
    double maxSpeed = getDrive().getSwerveDrive().getMaximumChassisVelocity();
    double scale    = inputMag * maxSpeed * PREDICT_DT;
    double dx = (fieldRelInputX / inputMag) * scale;
    double dy = (fieldRelInputY / inputMag) * scale;

    Translation2d current   = getDrive().getPose().getTranslation();
    Translation2d predicted = new Translation2d(current.getX() + dx, current.getY() + dy);

    // Lock if the predicted point lands outside the boundary on either axis
    boolean outsideX = Math.abs(predicted.getX() - centre.getX()) > half;
    boolean outsideY = Math.abs(predicted.getY() - centre.getY()) > half;

    wheelsLocked = outsideX || outsideY;
    return wheelsLocked;
  }

  /**
   * Returns a speed scalar in [MIN_SCALAR, 1.0] for Drive to multiply
   * translation velocity by.  This is the soft slowdown layer — runs
   * independently of the hard lock above.
   */
  public double getSpeedScalar() {
    double dist = getDistanceToNearestWall();
    if (dist >= SLOWDOWN_ZONE_M) return 1.0;
    if (dist <= 0.0)             return MIN_SCALAR;
    double t = dist / SLOWDOWN_ZONE_M;
    return MIN_SCALAR + t * (1.0 - MIN_SCALAR);
  }

  /**
   * Signed distance to the nearest wall (metres).
   * Positive = inside boundary; negative = already outside.
   */
  public double getDistanceToNearestWall() {
    Translation2d pos = getDrive().getPose().getTranslation();
    double distLeft   = (pos.getX() - centre.getX()) + half;
    double distRight  = half - (pos.getX() - centre.getX());
    double distBottom = (pos.getY() - centre.getY()) + half;
    double distTop    = half - (pos.getY() - centre.getY());
    return Math.min(Math.min(distLeft, distRight), Math.min(distBottom, distTop));
  }

  /** True when the robot is fully inside the boundary box. */
  public boolean isInsideBoundary() {
    Translation2d pos = getDrive().getPose().getTranslation();
    return Math.abs(pos.getX() - centre.getX()) <= half
        && Math.abs(pos.getY() - centre.getY()) <= half;
  }

  // ── Singleton ─────────────────────────────────────────────────────────────────

  private static Boundary instance = null;

  public static Boundary getInstance() {
    if (instance == null) instance = new Boundary();
    return instance;
  }
}