/*
 * Greedy Limelight 3A ball chase - camera-relative, no odometry.
 *
 * Hold gamepad1 RIGHT BUMPER to run the auto chase. Release it and you have normal
 * manual driving (left stick = drive/strafe, right stick X = turn, left bumper = intake).
 * Releasing the bumper is your instant abort.
 *
 * While the bumper is held, every loop it:
 *   1. Grabs the latest neural-detector result (skips stale ones)
 *   2. Picks a ball from allowedClasses, STICKING with whichever detection is closest to the
 *      one it was already tracking (target lock) so two visible balls don't cause flip-flopping.
 *      Falls back to nearest (lowest ty) when the locked ball hasn't been seen in a while.
 *   3. AIMS with a P controller on tx, and only drives forward once roughly aimed
 *   4. Estimates floor distance from ty:  dist = (CAM_H - BALL_H) / tan(CAM_PITCH - ty)
 *   5. Stops at STOP_DIST, dwells so the intake can grab, then searches again
 *   6. If the ball vanishes at close range (it goes under/behind the intake), it coasts
 *      straight forward for COAST_MS instead of spinning away, then does the pickup dwell
 *
 * Conventions used everywhere below:  fwd + = forward,  strafe + = RIGHT,  turn + = CLOCKWISE.
 * Limelight tx is + when the target is to the RIGHT, so turn = +tx * gain.
 *
 * Class order on the 3A model: 0 = yellow_pollen (neutral), 1 = red_nectar, 2 = blue_nectar.
 * CONFIRM this against the Limelight web UI's pipeline label list before trusting it - this
 * comment is not checked against the model at runtime.
 *
 * BEFORE FIRST RUN: put the robot on blocks and check that (a) all wheels spin forward for
 * +fwd and (b) a ball on your right makes the robot turn right. Fix motor directions if not.
 */
package org.firstinspires.ftc.teamcode.opModes;

import com.google.firebase.crashlytics.buildtools.reloc.org.apache.http.client.methods.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@TeleOp(name = "Ball Chase (3A Greedy)", group = "Hive Vision")
public class Test extends LinearOpMode {

    /* ---- limelight / geometry ---- */
    private static final String LIMELIGHT_NAME = "limelight";
    public static int    PIPELINE_INDEX = 0;      // your neural detector pipeline
    public static double CAM_PITCH_DEG  = 25.0;   // camera tilt BELOW horizontal
    public static double CAM_H          = 9.2;    // camera lens height above floor, in
    private static final double BALL_H         = 3.0;    // ball center height above floor, in

    /* ---- detection gating ---- */
    // Class ids - CONFIRM against the Limelight pipeline's label list, not just this comment.
    private static final int CLASS_YELLOW_NEUTRAL = 0;
    private static final int CLASS_RED   = 1;
    private static final int CLASS_BLUE  = 2;

    // Picked during init (see selectAlliance()). Default = "colored only": both alliance
    // colors, never the neutral yellow ball. D-pad in init can change this before start.
    private Set<Integer> allowedClasses = new HashSet<>(Arrays.asList(CLASS_RED, CLASS_BLUE));

    public static double MIN_CONF         = 0.44; // CHECK telemetry: is confidence 0-1 or 0-100 on your firmware?
    private static final long   MAX_STALENESS_US = 120_000; // ignore results older than 120 ms.
    // Limelight reports staleness in MICROSECONDS.

    /* ---- target lock (stops flip-flopping when 2+ balls are visible) ---- */
    public static double LOCK_GATE_DEG = 12.0;    // max frame-to-frame angular jump to count as "same ball"
    public static long   LOCK_LOST_MS  = 300;     // drop the lock if it isn't matched for this long

    /* ---- approach ---- */
    public static double STOP_DIST    = 2.0;     // camera-to-ball floor distance at pickup, in
    private static final double AIM_TOL_DEG  = 5.0;      // "aimed enough" to pick up
    public static double DRIVE_MIN_TX = 15.0;    // beyond this many degrees off, turn in place only
    public static double DRIVE_KP     = 0.03;     // forward power per inch of distance error
    public static double MIN_FWD      = 0.15;     // overcome static friction
    public static double MAX_FWD      = 0.7;

    /* ---- turning ---- */
    public static double TURN_KP      = 0.025;    // power per degree of tx
    public static double MIN_TURN     = 0.08;     // friction floor when outside the tolerance
    public static double MAX_TURN     = 0.6;
    public static double SEARCH_TURN  = 0.30;     // clockwise scan when nothing is visible

    /* ---- close-range coast + pickup ---- */
    public static double COAST_MAX_DIST = 26.0;   // only coast if last seen within this, in
    public static double COAST_POWER    = 0.25;
    public static long   COAST_MS       = 450;
    public static long   PICKUP_DWELL_MS = 600;
    public static double INTAKE_POWER   = 1.0;

    /* ---- hardware names ---- */
    private static final String MOTOR_LF = "leftFront";
    private static final String MOTOR_RF = "rightFront";
    private static final String MOTOR_LB = "leftBack";
    private static final String MOTOR_RB = "rightBack";
    private static final String INTAKE   = "intake";     // optional; skipped if not in the config

    private enum State { SEARCHING, CHASING, COASTING, PICKUP }

    private Limelight3A limelight;
    private DcMotor lf, rf, lb, rb;
    private DcMotor intake;                              // may be null
    private State state = State.SEARCHING;

    private final ElapsedTime lastSeen = new ElapsedTime();
    private final ElapsedTime pickupTimer = new ElapsedTime();
    private double lastSeenDist = Double.MAX_VALUE;

    // Target lock: which ball we're currently committed to, by its last known angles.
    private Double lockTx = null, lockTy = null;
    private final ElapsedTime lockAge = new ElapsedTime();

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, MOTOR_LF);
        rf = hardwareMap.get(DcMotor.class, MOTOR_RF);
        lb = hardwareMap.get(DcMotor.class, MOTOR_LB);
        rb = hardwareMap.get(DcMotor.class, MOTOR_RB);
        intake = hardwareMap.tryGet(DcMotor.class, INTAKE);

        // Positive power = forward on every wheel. Flip these if your wheels disagree.
        // NOTE: rf=REVERSE / rb=FORWARD is asymmetric within the right side - carried over as-is
        // from your file. Confirm that's intentional (e.g. a physically flipped gearbox) and not
        // a leftover edit, since it will otherwise make the robot curve instead of drive straight.
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{lf, rf, lb, rb}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.setPollRateHz(100);
        limelight.start();

        selectAlliance();

        telemetry.addLine("Ready. Hold RIGHT BUMPER to auto-chase; release to drive manually.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                autoChase();
            } else {
                state = State.SEARCHING;
                lockTx = null;
                lockTy = null;
                manualDrive();
            }
            telemetry.addData("state", state);
            telemetry.update();
        }

        limelight.stop();
        drive(0, 0, 0);
        setIntake(0);
    }

    /* ------------------------------------------------------------------ */

    /**
     * D-pad UP = red only, DOWN = blue only, LEFT = both colors / no yellow (default), RIGHT = all
     * three classes including yellow. Runs in the init loop, before waitForStart(), so this has no
     * effect once the match starts - lock it in before pressing start.
     */
    private void selectAlliance() {
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up)    allowedClasses = setOf(CLASS_RED);
            if (gamepad1.dpad_down)  allowedClasses = setOf(CLASS_BLUE);
            if (gamepad1.dpad_left)  allowedClasses = setOf(CLASS_RED, CLASS_BLUE);
            if (gamepad1.dpad_right) allowedClasses = setOf(CLASS_RED, CLASS_BLUE, CLASS_YELLOW_NEUTRAL);

            telemetry.addLine("Pick target color(s), then press start:");
            telemetry.addLine("  dpad UP = red only   DOWN = blue only");
            telemetry.addLine("  LEFT = red+blue (no yellow)   RIGHT = all colors incl. yellow");
            telemetry.addData("current", describeAllowed());
            telemetry.update();
        }
    }

    private String describeAllowed() {
        if (allowedClasses.size() == 1 && allowedClasses.contains(CLASS_RED)) return "RED only";
        if (allowedClasses.size() == 1 && allowedClasses.contains(CLASS_BLUE)) return "BLUE only";
        if (allowedClasses.size() == 2) return "RED + BLUE (no yellow)";
        return "RED + BLUE + YELLOW";
    }

    private static Set<Integer> setOf(Integer... ids) {
        return new HashSet<>(Arrays.asList(ids));
    }

    /* ------------------------------------------------------------------ */

    private void autoChase() {
        // Pickup dwell: hold still, run the intake, then go look for the next ball.
        if (state == State.PICKUP) {
            drive(0, 0, 0);
            setIntake(INTAKE_POWER);
            telemetry.addData("drive", "pickup - grabbing");
            if (pickupTimer.milliseconds() >= PICKUP_DWELL_MS) {
                lastSeenDist = Double.MAX_VALUE;
                lockTx = null;
                lockTy = null;
                state = State.SEARCHING;
            }
            return;
        }

        LLResultTypes.DetectorResult target = findTarget();

        if (target != null) {
            double tx = target.getTargetXDegrees();
            double ty = target.getTargetYDegrees();
            double dist = groundRange(ty);

            lastSeen.reset();
            lastSeenDist = dist;

            telemetry.addData("ball", "tx=%+.1f  ty=%+.1f  dist=%.1f in  conf=%.2f",
                    tx, ty, dist, target.getConfidence());

            boolean aimed = Math.abs(tx) < AIM_TOL_DEG;

            if (dist <= STOP_DIST && aimed) {
                state = State.PICKUP;
                pickupTimer.reset();
                drive(0, 0, 0);
                return;
            }

            // Turn: P control, clockwise-positive, with a small floor outside the tolerance.
            double turn = Range.clip(tx * TURN_KP, -MAX_TURN, MAX_TURN);
            if (!aimed && Math.abs(turn) < MIN_TURN) {
                turn = Math.copySign(MIN_TURN, tx);
            }

            // Forward: only once roughly pointed at the ball, and only until STOP_DIST.
            double fwd = 0.0;
            if (dist > STOP_DIST && Math.abs(tx) < DRIVE_MIN_TX) {
                fwd = Range.clip((dist - STOP_DIST) * DRIVE_KP, MIN_FWD, MAX_FWD);
            }

            state = State.CHASING;
            setIntake(INTAKE_POWER);
            drive(fwd, 0, turn);
            telemetry.addData("drive", "fwd=%.2f turn=%.2f", fwd, turn);
            return;
        }

        // ---- no target this frame ----
        boolean wasClose = (state == State.CHASING || state == State.COASTING)
                && lastSeenDist <= COAST_MAX_DIST;

        if (wasClose) {
            if (lastSeen.milliseconds() < COAST_MS) {
                // Ball dropped out of view right at the intake: keep going straight.
                state = State.COASTING;
                setIntake(INTAKE_POWER);
                drive(COAST_POWER, 0, 0);
                telemetry.addData("ball", "lost close - coasting");
            } else {
                state = State.PICKUP;
                pickupTimer.reset();
                drive(0, 0, 0);
            }
            return;
        }

        state = State.SEARCHING;
        setIntake(0);
        drive(0, 0, SEARCH_TURN);
        telemetry.addData("ball", "no target - scanning");
    }

    /**
     * Picks which ball to chase this frame. If we were already locked onto one, we stick with
     * whichever detection is angularly closest to where it was last frame (within LOCK_GATE_DEG),
     * even if a different ball is now technically nearer - this is what stops the robot flipping
     * back and forth when two balls are visible at similar range. Only when the locked ball can't
     * be matched (picked up, occluded, out of frame) for LOCK_LOST_MS do we drop the lock and
     * fall back to nearest (lowest ty).
     */
    private LLResultTypes.DetectorResult findTarget() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> dets =
                (result == null || result.getStaleness() > MAX_STALENESS_US) ? null : result.getDetectorResults();

        if (dets == null || dets.isEmpty()) {
            dropStaleLock();
            return null;
        }

        LLResultTypes.DetectorResult nearest = null;          // fallback: lowest ty = closest on floor
        LLResultTypes.DetectorResult sameAsLock = null;        // best match to the ball we're already on
        double bestMatchDist = Double.MAX_VALUE;

        for (LLResultTypes.DetectorResult d : dets) {
            if (!allowedClasses.contains(d.getClassId())) continue;
            if (d.getConfidence() < MIN_CONF) continue;

            if (nearest == null || d.getTargetYDegrees() < nearest.getTargetYDegrees()) {
                nearest = d;
            }
            if (lockTx != null) {
                double jump = Math.hypot(d.getTargetXDegrees() - lockTx, d.getTargetYDegrees() - lockTy);
                if (jump < bestMatchDist) {
                    bestMatchDist = jump;
                    sameAsLock = d;
                }
            }
        }

        if (nearest == null) {           // no confident detections at all this frame
            dropStaleLock();
            return null;
        }

        LLResultTypes.DetectorResult chosen =
                (sameAsLock != null && bestMatchDist <= LOCK_GATE_DEG) ? sameAsLock : nearest;

        lockTx = chosen.getTargetXDegrees();
        lockTy = chosen.getTargetYDegrees();
        lockAge.reset();
        return chosen;
    }

    /** If the locked ball hasn't been matched in a while, release the lock so we can pick freely again. */
    private void dropStaleLock() {
        if (lockTx != null && lockAge.milliseconds() > LOCK_LOST_MS) {
            lockTx = null;
            lockTy = null;
        }
    }

    /** Floor distance camera -> ball from the vertical angle. Big number if at/above the horizon. */
    private double groundRange(double tyDeg) {
        double depression = CAM_PITCH_DEG - tyDeg;      // angle below horizontal to the ball
        if (depression < 1.0) return 999.0;
        return (CAM_H - BALL_H) / Math.tan(Math.toRadians(depression));
    }

    /* ------------------------------------------------------------------ */

    private void manualDrive() {
        double fwd    = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;
        drive(fwd, strafe, turn);
        setIntake(gamepad1.left_bumper ? INTAKE_POWER : 0);
    }

    /** Mecanum: fwd + = forward, strafe + = right, turn + = clockwise. Powers are normalized. */
    private void drive(double fwd, double strafe, double turn) {
        double pLf = fwd + strafe + turn;
        double pRf = fwd - strafe - turn;
        double pLb = fwd - strafe + turn;
        double pRb = fwd + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(pLf), Math.abs(pRf)),
                        Math.max(Math.abs(pLb), Math.abs(pRb))));

        lf.setPower(pLf / max);
        rf.setPower(pRf / max);
        lb.setPower(pLb / max);
        rb.setPower(pRb / max);
    }

    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }
}