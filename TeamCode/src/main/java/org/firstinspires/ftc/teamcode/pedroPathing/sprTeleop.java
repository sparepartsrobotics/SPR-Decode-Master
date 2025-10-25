package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;
@Configurable
@TeleOp
public class sprTeleop extends OpMode {
    private Follower follower;
    private HuskyLens huskyLens;
    private Servo fanRotate, cam;
    private DcMotor outtake1, outtake2;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double currPosFan = .15;
    private double fastModeMultiplier = .6;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        MecanumConstants drive = new MecanumConstants();
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        fanRotate = hardwareMap.get(Servo.class, "fanRotate");
        cam = hardwareMap.get(Servo.class, "cam");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        fanRotate.setPosition(currPosFan);
        cam.setPosition(0);
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * fastModeMultiplier,
                    -gamepad1.left_stick_x * fastModeMultiplier,
                    -gamepad1.right_stick_x * fastModeMultiplier,
                    true // Robot Centric
            );

        //Automated PathFollowing
        //Stop automated following if the follower is done
        //Optional way to change slow mode strength
        if (gamepad1.rightStickButtonWasPressed()) {
            fastModeMultiplier += 0.25;
        }
        if(gamepad1.rightBumperWasPressed()){
            currPosFan += .05;
            fanRotate.setPosition(currPosFan);
        }
        if(gamepad1.leftBumperWasPressed()){
            currPosFan -= .05;
            fanRotate.setPosition(currPosFan);
        }

        if(gamepad1.aWasPressed()){
            currPosFan += .05;
            fanRotate.setPosition(currPosFan);
        }
        if(gamepad1.xWasPressed()){
            cam.setPosition(.5);
        }
        if(gamepad1.bWasPressed()){
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake1.setPower(.75);
            outtake2.setPower(.75);
        }
        if(gamepad1.yWasPressed()){
            outtake1.setPower(0);
            outtake2.setPower(0);
        }
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}