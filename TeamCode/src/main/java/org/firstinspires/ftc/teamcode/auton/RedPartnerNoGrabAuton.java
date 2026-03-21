package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.NewBotTeleopBlueTest;

@Autonomous(name = "RedPartnerNoGrabAuton")
public class RedPartnerNoGrabAuton extends OpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex, intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private VoltageSensor batteryVoltageSensor;

    private final Timer shootTimer = new Timer();
    private final Timer loopTimer = new Timer();
    private final Timer intakeTimer = new Timer();

    private static final double COREHEX_POWER = 1;
    private static final double SHOOT_TIME = 4.5;

    // Reduced time for the small CoreHex push to fit 3 pieces
    private static final double INTAKE_WAIT_TIME = 0.6;
    private static final double INTAKE_WAIT_TIME1 = 0.3;

    // Auto-Aim Variables from TeleOp
    private static final double GOAL_X = 135.5;
    private static final double GOAL_Y = 140;
    public static double MAX_MOTOR_VELOCITY = 1600.0;
    public static double FLYWHEEL_P = 0.8;
    private double targetFlywheelSpeed = 0;

    private final Pose startPose =
            new Pose(53.788, 6.771, Math.toRadians(90)).mirror();

    // Added path5 back in to match the 3-path intake sequence of 6-8
    private PathChain path1, path2, path3, path4, path5,
            path6, path7, path8, path9, path10, path11, path12, path13, path14;

    private enum State {
        PATH_1, PATH_1_WAIT, SHOOT_1,
        PATH_2, PATH_2_WAIT,
        PATH_3, PATH_3_WAIT,
        PATH_5, PATH_5_WAIT, INTAKE_WAIT_5,
        PATH_4, PATH_4_WAIT, SHOOT_2,
        PATH_6, PATH_6_WAIT,
        PATH_7, PATH_7_WAIT,
        PATH_8, PATH_8_WAIT, INTAKE_WAIT_8,
        PATH_9, PATH_9_WAIT, SHOOT_3,
        PATH_10, PATH_10_WAIT,
        PATH_11, PATH_11_WAIT,
        PATH_12, PATH_12_WAIT, INTAKE_WAIT_12,
        PATH_13, PATH_13_WAIT, SHOOT_4,
        PATH_14, PATH_14_WAIT,
        DONE
    }

    private State state = State.PATH_1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        curry = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        intake = hardwareMap.get(DcMotor.class, "intake");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        curry.setDirection(DcMotorEx.Direction.FORWARD);
        coreHex.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        curry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Changed to RUN_WITHOUT_ENCODER for custom PID/Feedforward to work correctly
        curry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buildPaths();
    }

    @Override
    public void start() {
        intake.setPower(0);
        follower.setMaxPower(1.0);
    }

    @Override
    public void loop() {
        follower.update();

        // Calculate auto-aim speed for the current frame
        double autoAimSpeed = flywheelSpeed(getDistanceToGoal());

        switch (state) {
            // --- CYCLE 1: SHOOT AFTER PATH 1 ---
            case PATH_1:
                targetFlywheelSpeed = autoAimSpeed;
                follower.followPath(path1, true);
                state = State.PATH_1_WAIT;
                break;
            case PATH_1_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_1;
                }
                break;
            case SHOOT_1:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_2;
                break;

            // --- CYCLE 2: INTAKE PATHS 2, 3, 5 (Mirrors 6-8), SHOOT AFTER PATH 4 ---
            case PATH_2:
                targetFlywheelSpeed = 0; // Save power
                intake.setPower(1); // Intake on through paths 2, 3, 5
                follower.followPath(path2, true);
                state = State.PATH_2_WAIT;
                loopTimer.resetTimer();
                break;
            case PATH_2_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy() && loopTimer.getElapsedTimeSeconds() > 0.3) state = State.PATH_3;
                break;

            case PATH_3:
                targetFlywheelSpeed = 0;
                follower.followPath(path3, true);
                state = State.PATH_3_WAIT;
                loopTimer.resetTimer();
                break;
            case PATH_3_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy() && loopTimer.getElapsedTimeSeconds() > 0.3) state = State.PATH_5;
                break;

            case PATH_5:
                targetFlywheelSpeed = 0;
                follower.followPath(path5, true);
                state = State.PATH_5_WAIT;
                loopTimer.resetTimer();
                break;
            case PATH_5_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy() && loopTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER); // Small bump to seat pieces
                    state = State.INTAKE_WAIT_5;
                }
                break;
            case INTAKE_WAIT_5:
                targetFlywheelSpeed = 0;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME1) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_4;
                }
                break;

            case PATH_4:
                targetFlywheelSpeed = autoAimSpeed; // Spin up on the way
                follower.followPath(path4, true);
                state = State.PATH_4_WAIT;
                break;
            case PATH_4_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_2;
                }
                break;
            case SHOOT_2:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_6;
                break;

            // --- CYCLE 3: INTAKE PATHS 6-8, SHOOT AFTER PATH 9 ---
            case PATH_6:
                targetFlywheelSpeed = 0; // Save power
                intake.setPower(1); // Intake on through paths 6-8
                follower.followPath(path6, true);
                state = State.PATH_6_WAIT;
                loopTimer.resetTimer();
                break;
            case PATH_6_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy() && loopTimer.getElapsedTimeSeconds() > 0.3) state = State.PATH_7;
                break;

            case PATH_7:
                targetFlywheelSpeed = 0;
                follower.followPath(path7, true);
                state = State.PATH_7_WAIT;
                loopTimer.resetTimer();
                break;
            case PATH_7_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy() && loopTimer.getElapsedTimeSeconds() > 0.3) state = State.PATH_8;
                break;

            case PATH_8:
                targetFlywheelSpeed = 0;
                follower.followPath(path8, true);
                state = State.PATH_8_WAIT;
                loopTimer.resetTimer();
                break;
            case PATH_8_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy() && loopTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER); // Small bump to seat pieces
                    state = State.INTAKE_WAIT_8;
                }
                break;
            case INTAKE_WAIT_8:
                targetFlywheelSpeed = 0;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME1) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_9;
                }
                break;

            case PATH_9:
                targetFlywheelSpeed = autoAimSpeed; // Spin up on the way
                follower.followPath(path9, true);
                state = State.PATH_9_WAIT;
                break;
            case PATH_9_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;
            case SHOOT_3:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_10;
                break;

            // --- CYCLE 4: INTAKE PATHS 10-12, SHOOT AFTER PATH 13 ---
            case PATH_10:
                targetFlywheelSpeed = 0; // Save power
                intake.setPower(1); // Intake on through paths 10-12
                follower.followPath(path10, true);
                state = State.PATH_10_WAIT;
                break;
            case PATH_10_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) state = State.PATH_11;
                break;

            case PATH_11:
                targetFlywheelSpeed = 0;
                follower.followPath(path11, true);
                state = State.PATH_11_WAIT;
                break;
            case PATH_11_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) state = State.PATH_12;
                break;

            case PATH_12:
                targetFlywheelSpeed = 0;
                follower.followPath(path12, true);
                state = State.PATH_12_WAIT;
                break;
            case PATH_12_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER); // Small bump to seat pieces
                    state = State.INTAKE_WAIT_12;
                }
                break;
            case INTAKE_WAIT_12:
                targetFlywheelSpeed = 0;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME1) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_13;
                }
                break;

            case PATH_13:
                targetFlywheelSpeed = autoAimSpeed; // Spin up on the way
                follower.followPath(path13, true);
                state = State.PATH_13_WAIT;
                break;
            case PATH_13_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_4;
                }
                break;
            case SHOOT_4:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_14;
                break;

            // --- FINISH ---
            case PATH_14:
                targetFlywheelSpeed = 0;
                follower.followPath(path14, true);
                state = State.PATH_14_WAIT;
                break;
            case PATH_14_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) state = State.DONE;
                break;

            case DONE:
                targetFlywheelSpeed = 0;
                coreHex.setPower(0);
                intake.setPower(0);
                stopDrive();
                NewBotTeleopBlueTest.startingPose = follower.getPose();
                break;
        }

        // Apply calculated velocity
        setShooterVelocity(targetFlywheelSpeed);

        telemetry.addData("State", state);
        telemetry.addData("Distance", getDistanceToGoal());
        telemetry.addData("Target Speed", targetFlywheelSpeed);
        telemetry.addData("Flywheel Velocity", curry.getVelocity());
        telemetry.update();
    }

    private boolean shootThree() {
        if (shootTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
            if(intakeTimer.getElapsedTimeSeconds() >= 1.5){
                intake.setPower(0.8);
            }

            // BOUNDS CHECK: Only feed if flywheel is up to speed dynamically
            double currentSpeed = Math.abs(curry.getVelocity());

            // Allow a +/- 20 unit margin for feeding to handle slight voltage dips
            if (currentSpeed >= targetFlywheelSpeed - 20 && currentSpeed <= targetFlywheelSpeed + 20) {
                coreHex.setPower(COREHEX_POWER);
            } else {
                coreHex.setPower(0); // Cut the feed if it bogs down or overshoots
            }

            return false;
        }

        coreHex.setPower(0);
        targetFlywheelSpeed = 0; // Shut off shooter between cycles to save power
        return true;
    }

    private double getDistanceToGoal() {
        Pose p = follower.getPose();
        return Math.hypot(GOAL_X - p.getX(), GOAL_Y - p.getY());
    }

    private double flywheelSpeed(double x) {
        return MathFunctions.clamp(
                0.0000227176 * Math.pow(x, 4) - 0.0106575 * Math.pow(x, 3) + 1.82035 * Math.pow(x, 2) - 129.17615 * x + 4077.24017,
                0, 2400
        );
    }

    private void setShooterVelocity(double targetVelocity) {
        if (targetVelocity == 0) {
            curry.setPower(0);
            return;
        }

        double currentVoltage = batteryVoltageSensor.getVoltage();
        if (currentVoltage <= 0) {
            currentVoltage = 12.0;
        }

        // Calculate feedforward and feedback exactly as in Teleop
        double basePower = targetVelocity / MAX_MOTOR_VELOCITY;
        double feedforward = basePower * (12.0 / currentVoltage);

        double currentSpeed = curry.getVelocity();
        double error = targetVelocity - currentSpeed;
        double feedback = error * FLYWHEEL_P;

        double totalPower = feedforward + feedback;
        curry.setPower(Math.max(0.0, Math.min(totalPower, 1.0)));
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(53.788, 6.771).mirror(),
                                new Pose(63.126, 18.307).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-115))
                .build();

        // path2 exactly matches path6
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307).mirror(),
                                new Pose(19.862, 8.078).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-180))
                .build();

        // path3 exactly matches path7
        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.862, 8.078).mirror(),
                                new Pose(23.235294117647058, 7.8489666136724985).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        // path5 exactly matches path8
        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(23.235294117647058, 7.8489666136724985).mirror(),
                                new Pose(19.862, 10.078).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        // path4 exactly matches path9 (Updated to return from path5's endpoint)
        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.862, 10.078).mirror(),
                                new Pose(63.126, 18.307).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-115))
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307).mirror(),
                                new Pose(19.862, 8.078).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-180))
                .build();

        path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.862, 8.078).mirror(),
                                new Pose(23.235294117647058, 7.8489666136724985).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(23.235294117647058, 7.8489666136724985).mirror(),
                                new Pose(19.862, 10.078).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.862, 10.078).mirror(),
                                new Pose(63.126, 18.307).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-115))
                .build();

        path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307).mirror(),
                                new Pose(19.862, 8.199).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-180))
                .build();

        path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.862, 8.199).mirror(),
                                new Pose(23.235294117647058, 9.870).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(23.235294117647058, 9.870).mirror(),
                                new Pose(19.862, 10.199).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path13 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.862, 10.199).mirror(),
                                new Pose(63.126, 18.307).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-115))
                .build();

        path14 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307).mirror(),
                                new Pose(43.388, 13.504).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-115))
                .build();
    }
}