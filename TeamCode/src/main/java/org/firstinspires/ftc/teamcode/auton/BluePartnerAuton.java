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

@Autonomous(name = "BluePartnerAuton")
public class BluePartnerAuton extends OpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex, intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Timer shootTimer = new Timer();
    private final Timer intakeTimer = new Timer();
    private final Timer intakeTimer1 = new Timer();

    private static final double COREHEX_POWER = 0.75;
    private static final double SHOOT_TIME = 3.0;
    private static final double INTAKE_WAIT_TIME = 0.5;

    private final Pose startPose =
            new Pose(53.788, 6.771, Math.toRadians(90));

    private PathChain path1, path2, path3, path4, path5,
            path6, path7, path8, path9, path10, path11, path12, path13, path14;

    private enum State {
        PATH_1, PATH_1_WAIT, PREPARE_SHOOT_1, SHOOT_1,
        PATH_2, PATH_2_WAIT, INTAKE_WAIT_2,
        PATH_3, PATH_3_WAIT,
        PATH_4, PATH_4_WAIT, INTAKE_WAIT_4,
        PATH_5, PATH_5_WAIT, PREPARE_SHOOT_2, SHOOT_2,
        PATH_6, PATH_6_WAIT, INTAKE_WAIT_6,
        PATH_7, PATH_7_WAIT,
        PATH_8, PATH_8_WAIT, INTAKE_WAIT_8,
        PATH_9, PATH_9_WAIT, PREPARE_SHOOT_3, SHOOT_3,
        PATH_10, PATH_10_WAIT, INTAKE_WAIT_10,
        PATH_11, PATH_11_WAIT,
        PATH_12, PATH_12_WAIT, INTAKE_WAIT_12,
        PATH_13, PATH_13_WAIT, PREPARE_SHOOT_4, SHOOT_4,
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
            // --- CYCLE 1 ---
            case PATH_1:
                follower.followPath(path1, true);
                state = State.PATH_1_WAIT;
                break;
            case PATH_1_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeTimer1.resetTimer();
                    state = State.PREPARE_SHOOT_1;
                }
                break;
            case PREPARE_SHOOT_1:
                if (shootTimer.getElapsedTimeSeconds() >= 2.0) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_1;
                }
                break;
            case SHOOT_1:
                if (shootThree()) state = State.PATH_2;
                break;

            case PATH_2:
                intake.setPower(1);
                follower.followPath(path2, true);
                state = State.PATH_2_WAIT;
                break;
            case PATH_2_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER); // Start coreHex wait
                    state = State.INTAKE_WAIT_2;
                }
                break;
            case INTAKE_WAIT_2:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0); // Intake OFF
                    state = State.PATH_3;
                }
                break;

            case PATH_3:
                follower.followPath(path3, true);
                state = State.PATH_3_WAIT;
                break;
            case PATH_3_WAIT:
                if (!follower.isBusy()) state = State.PATH_4;
                break;

            case PATH_4:
                intake.setPower(1); // Intake ON during path
                follower.followPath(path4, true);
                state = State.PATH_4_WAIT;
                break;
            case PATH_4_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    state = State.INTAKE_WAIT_4;
                }
                break;
            case INTAKE_WAIT_4:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_5;
                }
                break;

            // --- CYCLE 2 ---
            case PATH_5:
                follower.followPath(path5, true);
                state = State.PATH_5_WAIT;
                break;
            case PATH_5_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeTimer1.resetTimer();
                    state = State.PREPARE_SHOOT_2;
                }
                break;
            case PREPARE_SHOOT_2:
                if (shootTimer.getElapsedTimeSeconds() >= 2.0) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_2;
                }
                break;
            case SHOOT_2:
                if (shootThree()) state = State.PATH_6;
                break;

            case PATH_6:
                intake.setPower(1); // Intake ON
                follower.followPath(path6, true);
                state = State.PATH_6_WAIT;
                break;
            case PATH_6_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    state = State.INTAKE_WAIT_6;
                }
                break;
            case INTAKE_WAIT_6:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_7;
                }
                break;

            case PATH_7:
                follower.followPath(path7, true);
                state = State.PATH_7_WAIT;
                break;
            case PATH_7_WAIT:
                if (!follower.isBusy()) state = State.PATH_8;
                break;

            case PATH_8:
                intake.setPower(1); // Intake ON
                follower.followPath(path8, true);
                state = State.PATH_8_WAIT;
                break;
            case PATH_8_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
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

            // --- CYCLE 3 ---
            case PATH_9:
                follower.followPath(path9, true);
                state = State.PATH_9_WAIT;
                break;
            case PATH_9_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeTimer1.resetTimer();
                    state = State.PREPARE_SHOOT_3;
                }
                break;
            case PREPARE_SHOOT_3:
                if (shootTimer.getElapsedTimeSeconds() >= 2.0) {
                    shootTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;
            case SHOOT_3:
                if (shootThree()) state = State.PATH_10;
                break;

            case PATH_10:
                intake.setPower(1); // Intake ON
                follower.followPath(path10, true);
                state = State.PATH_10_WAIT;
                break;
            case PATH_10_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
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
                intake.setPower(1); // Intake ON
                follower.followPath(path12, true);
                state = State.PATH_12_WAIT;
                break;
            case PATH_12_WAIT:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
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

            // --- CYCLE 4 ---
            case PATH_13:
                follower.followPath(path13, true);
                state = State.PATH_13_WAIT;
                break;
            case PATH_13_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    intakeTimer1.resetTimer();
                    state = State.PREPARE_SHOOT_4;
                }
                break;
            case PREPARE_SHOOT_4:
                if (shootTimer.getElapsedTimeSeconds() >= 2.0) {
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
                NewBotTeleopBlue.startingPose = follower.getPose();
                stopDrive();
                break;
        }
    }

    private void prepareShooter() {
        curry.setVelocity(2300);
    }

    private boolean shootThree() {
        if (shootTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
            if(intakeTimer1.getElapsedTimeSeconds() < 2){
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
                                new Pose(11.365, 8.672)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.365, 8.672),
                                new Pose(21.048, 8.737)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.048, 8.737),
                                new Pose(11.857, 10.392)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.857, 10.392),
                                new Pose(63.126, 18.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307),
                                new Pose(11.365, 8.672)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.365, 8.672),
                                new Pose(21.048, 8.672)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.048, 8.672),
                                new Pose(11.857, 10.392)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.857, 10.392),
                                new Pose(63.126, 18.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307),
                                new Pose(11.365, 8.672)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.365, 8.672),
                                new Pose(21.048, 8.737)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.048, 8.737),
                                new Pose(11.857, 10.392)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path13 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.857, 10.392),
                                new Pose(63.126, 18.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        path14 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.126, 18.307),
                                new Pose(44.532, 15.348)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(115))
                .build();
    }
}