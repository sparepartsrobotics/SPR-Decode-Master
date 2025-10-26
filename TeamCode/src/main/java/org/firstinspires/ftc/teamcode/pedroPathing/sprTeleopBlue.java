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
public class sprTeleopBlue extends OpMode {
    private Follower follower;
    private HuskyLens huskyLens;
    private Servo fanRotate, cam;
    private DcMotor outtake1, outtake2, intake;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive, isCam;
    private Supplier<PathChain> pathChain1, pathChain2;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double currPosFan = .16, camPos = 1, currRelease=.1;


    private int count = 0;
    private double fastModeMultiplier = .6;
    @Override
    public void init() {
        BlueAuto x = new BlueAuto();
        follower = Constants.createFollower(hardwareMap);
        MecanumConstants drive = new MecanumConstants();
        follower.setStartingPose(x.getFinalPose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        fanRotate = hardwareMap.get(Servo.class, "fanRotate");
        cam = hardwareMap.get(Servo.class, "cam");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pathChain1 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(315), 0.8))
                .build();
        pathChain2 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 24))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(300), 0.8))
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
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * fastModeMultiplier,
                    -gamepad1.left_stick_x * fastModeMultiplier,
                    -gamepad1.right_stick_x * fastModeMultiplier,
                    true // Robot Centric
            );
            if (gamepad1.rightStickButtonWasPressed()) {
                fastModeMultiplier += 0.25;
            }
            if(gamepad1.dpadUpWasPressed()){
                cam.setPosition(camPos);
                if(camPos == 1){
                    camPos = 0;
                }
                else if(camPos == 0){
                    camPos = 1;
                }
            }
            if(gamepad1.dpadRightWasPressed()){
                fanRotate.getController().pwmDisable();
            }
            if(gamepad1.dpadLeftWasPressed()){
                fanRotate.getController().pwmEnable();
            }

            if(gamepad1.rightBumperWasPressed()){
                if(currPosFan < .37){
                    currPosFan += .11;
                    fanRotate.setPosition(currPosFan);
                }
            }
            if(gamepad1.leftBumperWasPressed()){
                currPosFan -= .11;
                fanRotate.setPosition(currPosFan);
            }

            if(gamepad1.aWasPressed()){
                outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setPower(.70);
                outtake2.setPower(.70);
            }
            if(gamepad1.bWasPressed()){
                outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setPower(.75);
                outtake2.setPower(.75);
            }
        }
        if(gamepad1.right_trigger > 0 && count == 0){
            intake.setPower(.5);
            count++;
        }
        else if(gamepad1.right_trigger > 0 && count == 1){
            intake.setPower(0);
            count--;
        }

        if(gamepad1.yWasPressed()){
            follower.followPath(pathChain1.get());
            automatedDrive = true;
        }
        if(gamepad1.xWasPressed()){
            follower.followPath(pathChain2.get());
            automatedDrive = true;
        }

        if (automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}