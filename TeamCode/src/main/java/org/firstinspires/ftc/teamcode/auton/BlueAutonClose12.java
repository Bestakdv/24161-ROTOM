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
import org.firstinspires.ftc.teamcode.teleop.NewBotTeleopBlue;

@Autonomous(name = "BlueAutonClose12")
public class BlueAutonClose12 extends OpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex, intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Timer shootTimer = new Timer();
    private final Timer intakeTimer = new Timer();
    private final Timer intakeShooterTimer = new Timer();
    private final Timer waitTimer = new Timer();

    private static final double COREHEX_POWER = 0.75;
    private static final double SHOOT_TIME = 3.0;
    private static final double INTAKE_WAIT_TIME = 0.5;

    private final Pose startPose =
            new Pose(27.49488054607509, 134.5529010238908, Math.toRadians(145));

    private PathChain path1, path2, path3, path4, path5,
            path6, path7, path8, path9, path10, path11, path12, path13;

    private enum State {
        PATH_1, PATH_1_WAIT, PREPARE_SHOOT_1, SHOOT_1,
        PATH_2, PATH_2_WAIT,
        PATH_3, PATH_3_WAIT, INTAKE_WAIT_3,
        PATH_4, PATH_4_WAIT, WAIT_4,
        PATH_5, PATH_5_WAIT,
        PATH_6, PATH_6_WAIT, SHOOT_2,
        PATH_7, PATH_7_WAIT, INTAKE_WAIT_7,
        PATH_8, PATH_8_WAIT,
        PATH_9, PATH_9_WAIT, SHOOT_3,
        PATH_10, PATH_10_WAIT, INTAKE_WAIT_10,
        PATH_11, PATH_11_WAIT,
        PATH_12, PATH_12_WAIT, SHOOT_4,
        PATH_13, PATH_13_WAIT,
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

        curry.setDirection(DcMotorEx.Direction.FORWARD);
        coreHex.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        curry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buildPaths();
    }

    @Override
    public void start() {
        intake.setPower(0);
        follower.setMaxPower(1);
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case PATH_1:
                follower.followPath(path1, true);
                state = State.PATH_1_WAIT;
                break;

            case PATH_1_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeShooterTimer.resetTimer();
                    state = State.PREPARE_SHOOT_1;
                }
                break;

            case PREPARE_SHOOT_1:
                if (shootTimer.getElapsedTimeSeconds() >= 1.5) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_1;
                }
                break;

            case SHOOT_1:
                if (shootThree()) state = State.PATH_2;
                break;

            case PATH_2:
                follower.followPath(path2, true);
                state = State.PATH_2_WAIT;
                break;

            case PATH_2_WAIT:
                if (!follower.isBusy()) state = State.PATH_3;
                break;

            case PATH_3:
                follower.followPath(path3, true); // 0.5 Power
                state = State.PATH_3_WAIT;
                break;

            case PATH_3_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    intake.setPower(1);
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
                follower.followPath(path4, 0.5,true);
                state = State.PATH_4_WAIT;
                break;

            case PATH_4_WAIT:
                if (!follower.isBusy()) {
                    waitTimer.resetTimer(); // Start the wait timer
                    state = State.WAIT_4;
                }
                break;

            case WAIT_4:
                // Wait for 0.5 seconds before moving to PATH_5
                if (waitTimer.getElapsedTimeSeconds() >= 1) {
                    state = State.PATH_5;
                }
                break;

            case PATH_5:
                follower.followPath(path5, true);
                state = State.PATH_5_WAIT;
                break;

            case PATH_5_WAIT:
                if (!follower.isBusy()) state = State.PATH_6;
                break;

            case PATH_6:
                follower.followPath(path6, true);
                state = State.PATH_6_WAIT;
                break;

            case PATH_6_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeShooterTimer.resetTimer();
                    state = State.SHOOT_2;
                }
                break;

            case SHOOT_2:
                if (shootThree()) state = State.PATH_7;
                break;

            case PATH_7:
                follower.followPath(path7, 0.5, true); // 0.5 Power
                state = State.PATH_7_WAIT;
                break;

            case PATH_7_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    intake.setPower(1);
                    state = State.INTAKE_WAIT_7;
                }
                break;

            case INTAKE_WAIT_7:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_8;
                }
                break;

            case PATH_8:
                follower.followPath(path8, true);
                state = State.PATH_8_WAIT;
                break;

            case PATH_8_WAIT:
                if (!follower.isBusy()) state = State.PATH_9;
                break;

            case PATH_9:
                follower.followPath(path9, true);
                state = State.PATH_9_WAIT;
                break;

            case PATH_9_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeShooterTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;

            case SHOOT_3:
                if (shootThree()) state = State.PATH_10;
                break;

            case PATH_10:
                follower.followPath(path10, 0.5, true);
                state = State.PATH_10_WAIT;
                break;

            case PATH_10_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    intake.setPower(1);
                    state = State.INTAKE_WAIT_10;
                }
                break;

            case INTAKE_WAIT_10:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_11;
                }
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
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeShooterTimer.resetTimer();
                    state = State.SHOOT_4;
                }
                break;

            case SHOOT_4:
                if (shootThree()) state = State.PATH_13;
                break;

            case PATH_13:
                follower.followPath(path13, true);
                state = State.PATH_13_WAIT;
                break;

            case PATH_13_WAIT:
                if (!follower.isBusy()) state = State.DONE;
                break;

            case DONE:
                curry.setPower(0);
                coreHex.setPower(0);
                intake.setPower(0);
                stopDrive();
                NewBotTeleopBlue.startingPose = follower.getPose();
                break;
        }
    }

    private void prepareShooter() {
        curry.setVelocity(1550);
    }

    private boolean shootThree() {
        if (shootTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
            if(intakeShooterTimer.getElapsedTimeSeconds() < 2){
                intake.setPower(0.75);
            }
            coreHex.setPower(COREHEX_POWER);
            intake.setPower(1);
            return false;
        }
        coreHex.setPower(0);
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
                                new Pose(27.495, 134.553),
                                new Pose(50.594, 93.010)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.594, 93.010),
                                new Pose(49.256, 59.201)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49.256, 59.201),
                                new Pose(22.962, 59.201)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.962, 59.201),
                                new Pose(16.451, 60.522)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.451, 60.522),
                                new Pose(49.256, 59.201)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49.256, 59.201),
                                new Pose(50.594, 93.010)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.594, 93.010),
                                new Pose(49.256, 83.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49.256, 83.352),
                                new Pose(22.962, 83.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.962, 83.352),
                                new Pose(50.594, 93.010)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                .build();

        path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.594, 93.010),
                                new Pose(49.256, 35.171)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49.256, 35.171),
                                new Pose(22.962, 35.171)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.962, 35.171),
                                new Pose(50.594, 93.010)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        path13 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.594, 93.010),
                                new Pose(51.765, 113.116)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();
    }
}