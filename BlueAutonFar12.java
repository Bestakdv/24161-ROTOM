package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.NewBotTeleopBlue;

@Autonomous(name = "BlueAutonFar12")
public class BlueAutonFar12 extends OpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex, intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo green;
    private GoBildaPinpointDriver pinpoint;

    private final Timer shootTimer = new Timer();

    private static final double COREHEX_POWER = 0.75;
    private static final double SHOOT_TIME = 3.0;

    private static final double GOAL_X = 9;
    private static final double GOAL_Y = 138;

    private final Pose startPose =
            new Pose(65.015, 6.875, Math.toRadians(90));

    private PathChain path1, path2, path3, path4, path5,
            path6, path7, path8, path9, path10, path11, path12;

    private enum State {
        PATH_1, PATH_1_WAIT, SHOOT_1,
        PATH_2, PATH_3, PATH_4,
        PATH_5, PATH_5_WAIT, SHOOT_2,
        PATH_6, PATH_7,
        PATH_8, PATH_8_WAIT, SHOOT_3,
        PATH_9, PATH_10,
        PATH_11, PATH_11_WAIT, SHOOT_4,
        PATH_12, PATH_12_WAIT,
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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        green = hardwareMap.get(Servo.class, "hood");

        curry.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        curry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO CHANGE PIDF ONCE TUNED
        PIDFCoefficients PIDF = new PIDFCoefficients(1,0,0,16);
        curry.setPIDFCoefficients(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, PIDF);

        pinpoint.resetPosAndIMU();
        pinpoint.update();

        buildPaths();
    }

    @Override
    public void start() {
        intake.setPower(0.75);
        curry.setVelocity(1900);
        green.setPosition(0.4);
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
                    state = State.SHOOT_1;
                }
                break;

            case SHOOT_1:
                if (shootThree()) state = State.PATH_2;
                break;

            case PATH_2:
                follower.followPath(path2, true);
                state = State.PATH_3;
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    follower.followPath(path3, 0.55, true);
                    state = State.PATH_4;
                }
                break;

            case PATH_4:
                if (!follower.isBusy()) {
                    follower.followPath(path4, 1, true);
                    state = State.PATH_5;
                }
                break;

            case PATH_5:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    state = State.PATH_5_WAIT;
                }
                break;

            case PATH_5_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    state = State.SHOOT_2;
                }
                break;

            case SHOOT_2:
                if (shootThree()) state = State.PATH_6;
                break;

            case PATH_6:
                follower.followPath(path6, true);
                state = State.PATH_7;
                break;

            case PATH_7:
                if (!follower.isBusy()) {
                    follower.followPath(path7, 0.55, true);
                    state = State.PATH_8;
                }
                break;

            case PATH_8:
                if (!follower.isBusy()) {
                    follower.followPath(path8, 1, true);
                    state = State.PATH_8_WAIT;
                }
                break;

            case PATH_8_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;

            case SHOOT_3:
                if (shootThree()) state = State.PATH_9;
                break;

            case PATH_9:
                follower.followPath(path9, true);
                state = State.PATH_10;
                break;

            case PATH_10:
                if (!follower.isBusy()) {
                    follower.followPath(path10, 0.55, true);
                    state = State.PATH_11;
                }
                break;

            case PATH_11:
                if (!follower.isBusy()) {
                    follower.followPath(path11, 1, true);
                    state = State.PATH_11_WAIT;
                }
                break;

            case PATH_11_WAIT:
                if (!follower.isBusy()) {
                    prepareShooter();
                    shootTimer.resetTimer();
                    state = State.SHOOT_4;
                }
                break;

            case SHOOT_4:
                if (shootThree()) state = State.PATH_12;
                break;

            case PATH_12:
                follower.followPath(path12, true);
                state = State.PATH_12_WAIT;
                break;

            case PATH_12_WAIT:
                if (!follower.isBusy()) state = State.DONE;
                break;

            case DONE:
                curry.setPower(0);
                coreHex.setPower(0);
                intake.setPower(0);
                stopDrive();
                NewBotTeleopBlue.startingPose=follower.getPose();
                break;
        }
    }

    private double getDistanceToGoal() {
        Pose p = follower.getPose();
        return Math.hypot(GOAL_X - p.getX(), GOAL_Y - p.getY());
    }

    private void prepareShooter() {
        double d = getDistanceToGoal();
        curry.setVelocity(flywheelSpeed(d));
        green.setPosition(hoodAngle(d));
    }

    private boolean shootThree() {
        if (shootTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
            coreHex.setPower(COREHEX_POWER);
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

    // TODO tune
    public static double flywheelSpeed(double x) {
        return MathFunctions.clamp(
                -0.0847688 * x * x + 15.36083 * x + 1225.70392,
                0, 1920
        );
    }

    public static double hoodAngle(double x) {
        return MathFunctions.clamp(
                -0.0847688 * x * x + 15.36083 * x + 1225.70392,
                0, 1
        );
    }

    private void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(65.015, 6.875),
                        new Pose(59.710, 15.433)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90), Math.toRadians(119))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(59.710, 15.433),
                        new Pose(39.402, 59.045)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(119), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(39.402, 59.045),
                        new Pose(18.202, 59.075)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(18.202, 59.075),
                        new Pose(13.294, 66.115)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), Math.toRadians(180))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(13.294, 66.115),
                        new Pose(49.667, 42.445),
                        new Pose(59.710, 15.433)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), Math.toRadians(119))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(59.710, 15.433),
                        new Pose(41.282, 34.638)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(119), Math.toRadians(180))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(41.282, 34.638),
                        new Pose(21.777, 35.187)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), Math.toRadians(180))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(21.777, 35.187),
                        new Pose(59.710, 15.433)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), Math.toRadians(119))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(59.710, 15.433),
                        new Pose(38.678, 83.071)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(119), Math.toRadians(180))
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(38.678, 83.071),
                        new Pose(21.473, 83.281)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), Math.toRadians(180))
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(21.473, 83.281),
                        new Pose(59.710, 15.433)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), Math.toRadians(119))
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(59.710, 15.433),
                        new Pose(53.255, 19.153)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(119), Math.toRadians(119))
                .build();
    }
}
