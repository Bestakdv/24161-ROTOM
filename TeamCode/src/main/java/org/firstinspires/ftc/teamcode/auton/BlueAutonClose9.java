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
import org.firstinspires.ftc.teamcode.teleop.NewBotTeleopBlue;

@Autonomous(name = "BlueAutonClose9")
public class BlueAutonClose9 extends OpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex, intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Timer shootTimer = new Timer();
    private final Timer coreTimer = new Timer();
    private final Timer intakeTimer = new Timer();
    private final Timer intakeTimer1 = new Timer();

    private static final double COREHEX_POWER = 0.9;
    private static final double SHOOT_TIME = 5.0;
    private static final double INTAKE_WAIT_TIME = 0.3;

    private static final double GOAL_X = 6.5;
    private static final double GOAL_Y = 138;
    private VoltageSensor batteryVoltageSensor;

    // Flywheel Tuning Variables
    private static final double MAX_MOTOR_VELOCITY = 2020.0;
    public static double FLYWHEEL_P = 0.0012;
    private double targetFlywheelSpeed = 0;

    private final Pose startPose =
            new Pose(27.49488054607509, 134.5529010238908, Math.toRadians(145));

    private PathChain path1, path2, path3, path4, path5,
            path6, path7, path8;

    private enum State {
        PATH_1, PATH_1_WAIT, SHOOT_1,
        PATH_2, PATH_2_WAIT,
        PATH_3, PATH_3_WAIT, INTAKE_WAIT_3,
        PATH_4, PATH_4_WAIT, SHOOT_2,
        PATH_5, PATH_5_WAIT,
        PATH_6, PATH_6_WAIT, INTAKE_WAIT_6,
        PATH_7, PATH_7_WAIT, SHOOT_3,
        PATH_8, PATH_8_WAIT,
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
        curry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buildPaths();
    }

    @Override
    public void start() {
        intake.setPower(0);
        follower.setMaxPower(0.85);
    }

    @Override
    public void loop() {
        follower.update();

        // Calculate the ideal speed based on distance for this frame
        double autoAimSpeed = flywheelSpeed(getDistanceToGoal());

        switch (state) {
            case PATH_1:
                targetFlywheelSpeed = autoAimSpeed; // Spin up on the way
                follower.followPath(path1);
                state = State.PATH_1_WAIT;
                break;

            case PATH_1_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer1.resetTimer();
                    coreTimer.resetTimer();
                    state = State.SHOOT_1;
                }
                break;

            case SHOOT_1:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_2;
                break;

            case PATH_2:
                targetFlywheelSpeed = -800; // Reverse intake speed
                coreHex.setPower(0.45);
                intake.setPower(1);
                follower.followPath(path2);
                state = State.PATH_2_WAIT;
                break;

            case PATH_2_WAIT:
                targetFlywheelSpeed = -800;
                if (!follower.isBusy()) state = State.PATH_3;
                break;

            case PATH_3:
                targetFlywheelSpeed = -800;
                follower.followPath(path3, true);
                state = State.PATH_3_WAIT;
                break;

            case PATH_3_WAIT:
                targetFlywheelSpeed = -800;
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(0.45);
                    state = State.INTAKE_WAIT_3;
                }
                break;

            case INTAKE_WAIT_3:
                targetFlywheelSpeed = -800;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
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
                    intakeTimer1.resetTimer();
                    coreTimer.resetTimer();
                    state = State.SHOOT_2;
                }
                break;

            case SHOOT_2:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_5;
                break;

            case PATH_5:
                targetFlywheelSpeed = 0; // Turn off flywheel during transit
                intake.setPower(0);
                coreHex.setPower(0);
                follower.followPath(path5, true);
                state = State.PATH_5_WAIT;
                break;

            case PATH_5_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) state = State.PATH_6;
                break;

            case PATH_6:
                targetFlywheelSpeed = -800; // Reverse intake speed
                coreHex.setPower(0.45);
                intake.setPower(1);
                follower.followPath(path6, true);
                state = State.PATH_6_WAIT;
                break;

            case PATH_6_WAIT:
                targetFlywheelSpeed = -800;
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(0.45);
                    state = State.INTAKE_WAIT_6;
                }
                break;

            case INTAKE_WAIT_6:
                targetFlywheelSpeed = -800;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    state = State.PATH_7;
                }
                break;

            case PATH_7:
                targetFlywheelSpeed = autoAimSpeed; // Spin up on the way
                follower.followPath(path7, true);
                state = State.PATH_7_WAIT;
                break;

            case PATH_7_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer1.resetTimer();
                    coreTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;

            case SHOOT_3:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.DONE;
                break;

            case DONE:
                targetFlywheelSpeed = 0;
                coreHex.setPower(0);
                intake.setPower(0);
                NewBotTeleopBlue.startingPose = follower.getPose();
                stopDrive();
                break;
        }

        // Apply the calculated speed consistently at the end of the loop
        setShooterVelocity(targetFlywheelSpeed);

        telemetry.addData("State", state);
        telemetry.addData("Target Speed", targetFlywheelSpeed);
        telemetry.addData("Current Velocity", curry.getVelocity());
        telemetry.update();
    }

    private boolean shootThree() {
        if (shootTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
            if (intakeTimer1.getElapsedTimeSeconds() < 0.6) {
                intake.setPower(1);
            }

            double currentVelocity = curry.getVelocity();

            // DYNAMIC BOUNDS: Checks if velocity is within ±20 of the calculated target
            boolean upToSpeed = currentVelocity > (targetFlywheelSpeed - 20) && currentVelocity < (targetFlywheelSpeed + 20);

            if (upToSpeed && coreTimer.getElapsedTimeSeconds() > 0.3) {
                coreHex.setPower(COREHEX_POWER);
            } else {
                coreHex.setPower(0);
            }
            return false;
        }
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

        // Allows us to use the P-controller for negative intaking speeds too
        double sign = Math.signum(targetVelocity);
        double absTarget = Math.abs(targetVelocity);
        double currentSpeed = Math.abs(curry.getVelocity());

        double basePower = absTarget / MAX_MOTOR_VELOCITY;
        double feedforward = basePower * (12.0 / currentVoltage);

        double error = absTarget - currentSpeed;
        double feedback = error * FLYWHEEL_P;

        double totalPower = feedforward + feedback;

        // Apply direction and clamp to [-1.0, 1.0]
        curry.setPower(sign * Math.max(0.0, Math.min(totalPower, 1.0)));
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
                                new Pose(52.536, 90.805)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(137))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(52.536, 90.805),
                                new Pose(46.0002662116041, 86.44333788395906)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(46.0002662116041, 86.44333788395906),
                                new Pose(25.5, 86.82)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(25.5, 86.82),
                                new Pose(52.536, 90.805)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(52.536, 90.805),
                                new Pose(47.003, 63.023890784982946)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(47.003, 63.023890784982946),
                                new Pose(18.030716723549485, 63.023890784982946)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18.030716723549485, 63.023890784982946),
                                new Pose(63.427662957074716, 93.5135135135135)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
    }
}
