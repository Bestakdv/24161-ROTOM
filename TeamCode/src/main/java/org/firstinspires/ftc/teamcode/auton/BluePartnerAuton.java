package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BluePartnerAuton")
public class BluePartnerAuton extends OpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex, intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Timer shootTimer = new Timer();
    private final Timer intakeTimer = new Timer();

    private static final double COREHEX_POWER = 1;
    private static final double SHOOT_TIME = 5.0;

    // Reduced time for the small CoreHex push to fit 3 pieces
    private static final double INTAKE_WAIT_TIME = 0.6;

    private final Pose startPose =
            new Pose(53.788, 6.771, Math.toRadians(90));

    // Removed path5
    private PathChain path1, path2, path3, path4,
            path6, path7, path8, path9, path10, path11, path12, path13, path14;

    private enum State {
        PATH_1, PATH_1_WAIT, SHOOT_1,
        PATH_2, PATH_2_WAIT,
        PATH_3, PATH_3_WAIT, INTAKE_WAIT_3,
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

        curry.setDirection(DcMotorEx.Direction.FORWARD);
        coreHex.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        curry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        switch (state) {
            // --- CYCLE 1: SHOOT AFTER PATH 1 ---
            case PATH_1:
                prepareShooter(); // Spin up on the way
                follower.followPath(path1, true);
                state = State.PATH_1_WAIT;
                break;
            case PATH_1_WAIT:
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_1;
                }
                break;
            case SHOOT_1:
                if (shootThree()) state = State.PATH_2;
                break;

            // --- CYCLE 2: INTAKE PATHS 2-3, SHOOT AFTER PATH 4 ---
            case PATH_2:
                follower.followPath(path2, true);
                state = State.PATH_2_WAIT;
                break;
            case PATH_2_WAIT:
                if (!follower.isBusy()) state = State.PATH_3;
                break;

            case PATH_3:
                intake.setPower(1);
                follower.followPath(path3, true);
                state = State.PATH_3_WAIT;
                break;
            case PATH_3_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER); // Start the small bump to seat the pieces
                    state = State.INTAKE_WAIT_3;
                }
                break;
            case INTAKE_WAIT_3:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_4;
                }
                break;

            case PATH_4:
                prepareShooter(); // Spin up on the way
                follower.followPath(path4, true);
                state = State.PATH_4_WAIT;
                break;
            case PATH_4_WAIT:
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_2;
                }
                break;
            case SHOOT_2:
                if (shootThree()) state = State.PATH_6;
                break;

            // --- CYCLE 3: INTAKE PATHS 6-8, SHOOT AFTER PATH 9 ---
            case PATH_6:
                intake.setPower(1); // Intake on through paths 6-8
                follower.followPath(path6, true);
                state = State.PATH_6_WAIT;
                break;
            case PATH_6_WAIT:
                if (!follower.isBusy()) state = State.PATH_7;
                break;

            case PATH_7:
                follower.followPath(path7, true);
                state = State.PATH_7_WAIT;
                break;
            case PATH_7_WAIT:
                if (!follower.isBusy()) state = State.PATH_8;
                break;

            case PATH_8:
                follower.followPath(path8, true);
                state = State.PATH_8_WAIT;
                break;
            case PATH_8_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER); // Small bump to seat pieces
                    state = State.INTAKE_WAIT_8;
                }
                break;
            case INTAKE_WAIT_8:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_9;
                }
                break;

            case PATH_9:
                prepareShooter(); // Spin up on the way
                follower.followPath(path9, true);
                state = State.PATH_9_WAIT;
                break;
            case PATH_9_WAIT:
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;
            case SHOOT_3:
                if (shootThree()) state = State.PATH_10;
                break;

            // --- CYCLE 4: INTAKE PATHS 10-12, SHOOT AFTER PATH 13 ---
            case PATH_10:
                intake.setPower(1); // Intake on through paths 10-12
                follower.followPath(path10, true);
                state = State.PATH_10_WAIT;
                break;
            case PATH_10_WAIT:
                if (!follower.isBusy()) state = State.PATH_11;
                break;

            case PATH_11:
                follower.followPath(path11, true);
                state = State.PATH_11_WAIT;
                break;
            case PATH_11_WAIT:
                if (!follower.isBusy()) state = State.PATH_12;
                break;

            case PATH_12:
                follower.followPath(path12, true);
                state = State.PATH_12_WAIT;
                break;
            case PATH_12_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER); // Small bump to seat pieces
                    state = State.INTAKE_WAIT_12;
                }
                break;
            case INTAKE_WAIT_12:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_13;
                }
                break;

            case PATH_13:
                prepareShooter(); // Spin up on the way
                follower.followPath(path13, true);
                state = State.PATH_13_WAIT;
                break;
            case PATH_13_WAIT:
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_4;
                }
                break;
            case SHOOT_4:
                if (shootThree()) state = State.PATH_14;
                break;

            // --- FINISH ---
            case PATH_14:
                follower.followPath(path14, true);
                state = State.PATH_14_WAIT;
                break;
            case PATH_14_WAIT:
                if (!follower.isBusy()) state = State.DONE;
                break;

            case DONE:
                curry.setPower(0);
                coreHex.setPower(0);
                intake.setPower(0);
                stopDrive();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Flywheel Velocity", curry.getVelocity());
        telemetry.update();
    }

    private void prepareShooter() {
        curry.setVelocity(1360);
    }

    private boolean shootThree() {
        if (shootTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
            intake.setPower(1);

            // BOUNDS CHECK: Only feed if flywheel is perfectly up to speed
            double currentSpeed = Math.abs(curry.getVelocity());
            if (currentSpeed >= 1360) {
                coreHex.setPower(COREHEX_POWER);
            } else {
                coreHex.setPower(0); // Cut the feed if it bogs down or overshoots
            }

            return false;
        }

        coreHex.setPower(0);
        curry.setPower(0); // Shut off shooter between cycles to save power
        return true;
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
                                new Pose(53.788, 6.771),
                                new Pose(63.126, 18.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307),
                                new Pose(45.494, 29.580)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(45.494, 29.580),
                                new Pose(22.819, 29.769)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.819, 29.769),
                                new Pose(63.126, 18.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307),
                                new Pose(13.862, 8.078)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.862, 8.078),
                                new Pose(15.235294117647058, 7.8489666136724985)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.235294117647058, 7.8489666136724985),
                                new Pose(14.091, 7.620)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.091, 7.620),
                                new Pose(63.126, 18.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307),
                                new Pose(15.401, 9.199)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.401, 9.199),
                                new Pose(25.493, 9.870)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(25.493, 9.870),
                                new Pose(15.401, 9.199)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path13 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.401, 9.199),
                                new Pose(63.126, 18.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        path14 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307),
                                new Pose(43.388, 13.504)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(115))
                .build();
    }
}
