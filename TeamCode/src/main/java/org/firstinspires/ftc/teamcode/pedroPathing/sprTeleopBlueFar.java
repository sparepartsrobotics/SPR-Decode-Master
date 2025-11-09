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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class sprTeleopBlueFar extends OpMode {
    private Follower follower;
    private HuskyLens huskyLens;
    private Servo fanRotate, cam, park1, park2;
    private DcMotorSimple outtake1, outtake2, intake, outtake3, rightFront, leftFront, rightRear, leftRear;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive, isCam;
    private Supplier<PathChain> pathChain1, pathChain2;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double currPosFan = .05, camPos = 1, currRelease=-.01;
    private double fanPos1 = .16, fanPos2 =  .27, fanPos3 =.38, fanPos4 = .49;
    private double upPos1 = .1, upPos2 =  .21, upPos3 =.32;
    private boolean x = true;
    private boolean x2 = true;
    private int count = 1, count3 = 1;
    private int count2 = 1;
    private double motorPower1 = .6;
    private double fastModeMultiplier = .3;

    @Override
    public void init() {
        BlueAuto1 x = new BlueAuto1();
        follower = Constants.createFollower(hardwareMap);
        MecanumConstants drive = new MecanumConstants();
        follower.setStartingPose(x.getFinalPose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        fanRotate = hardwareMap.get(Servo.class, "fanRotate");
        cam = hardwareMap.get(Servo.class, "cam");
        park1 = hardwareMap.get(Servo.class, "park1");
        park2 = hardwareMap.get(Servo.class, "park2");
        outtake1 = hardwareMap.get(DcMotorSimple.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorSimple.class, "outtake2");
        outtake3 = hardwareMap.get(DcMotorSimple.class, "outtake3");
        leftFront = hardwareMap.get(DcMotorSimple.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorSimple.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorSimple.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorSimple.class, "rightFront");
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        if(x != null)
        pathChain1 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(58, 70))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(312), 0.8))
                .build();
        pathChain2 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(57, 22))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(295), 0.8))
                .build();

    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        fanRotate.setPosition(currPosFan);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
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
                fastModeMultiplier = 0.75;
            }
            if(gamepad1.rightStickButtonWasReleased()){
                fastModeMultiplier = .3;
            }
            if(gamepad1.leftStickButtonWasPressed()){
                outtake1.setPower(0);
                outtake2.setPower(0);
                outtake3.setPower(0);
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


            if(gamepad1.rightBumperWasPressed()){
                if(count == 3 && count2 == 1){
                    fanRotate.setPosition(currPosFan);
                    count = 1;
                }
                else if(count == 1){
                    fanRotate.setPosition(fanPos1);
                    count++;
                }
                else if(count == 2){
                    fanRotate.setPosition(fanPos2);
                    count++;
                }
                else if(count==3){
                    fanRotate.setPosition(fanPos3);
                    count++;
                }
                else if(count == 4){
                    fanRotate.setPosition(fanPos4);
                    count = 1;
                }
            }
            if(gamepad1.leftBumperWasPressed()){
                fanRotate.setPosition(fanRotate.getPosition() - .11);
            }
            if(gamepad1.dpadRightWasPressed()){
                if(count2 ==1){
                    fanRotate.setPosition(upPos3);
                    count2++;
                }
                else if(count2 == 2) {
                    fanRotate.setPosition(upPos2);
                    count2++;
                }
                else if(count2==3){
                    fanRotate.setPosition(upPos1);
                    count2 = 1;

                }
            }
            if(gamepad1.dpadDownWasPressed()){
                park1.setPosition(.75);
                park2.setPosition(.75);
            }

            if(gamepad1.dpadLeftWasPressed()){
                fanRotate.setPosition(fanRotate.getPosition() - .11);
            }
//            if(gamepad1.rightBumperWasPressed()){
//                currPosFan+=.11;
//                fanRotate.setPosition(currPosFan);
//            } else if (gamepad1.leftBumperWasPressed()) {
//                currPosFan-=.11;
//                fanRotate.setPosition(currPosFan);
//            }
//            if(gamepad1.leftBumperWasPressed()){
//                currPosFan -= .11;
//                fanRotate.setPosition(currPosFan);
//            }

            if(gamepad1.aWasPressed()){
                motorPower1=.6;
                outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake3.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setPower(motorPower1);
                outtake2.setPower(motorPower1);
                outtake3.setPower(motorPower1);
            }
            if(gamepad1.bWasPressed()){
                motorPower1=.6;
                outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake3.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setPower(motorPower1+.05);
                outtake2.setPower(motorPower1+.05);
                outtake3.setPower(motorPower1+.05);
            }
        }
        if(automatedDrive){
            if(gamepad1.dpadRightWasPressed()){
                if(count2 == 1){
                    fanRotate.setPosition(upPos1);
                    count2++;
                    x2 = false;
                }
                else if(count2 == 2 && x2){
                    fanRotate.setPosition(upPos2);
                    count2--;
                }
                else if(count2 == 2 && !x2){
                    fanRotate.setPosition(upPos2);
                    count2++;
                }
                else if(count2==3){
                    fanRotate.setPosition(upPos3);
                    count2--;
                    x2 = true;
                }
            }
            if(gamepad1.yWasPressed()){
                automatedDrive = false;
                follower.breakFollowing();
                follower.startTeleopDrive();
            }

            if(gamepad1.xWasPressed()){
                automatedDrive = false;
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
        }
        if(gamepad1.left_trigger > 0){
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1);
        }
        else if(gamepad1.left_trigger <= 0){
            intake.setPower(0);
        }
        if(gamepad1.right_trigger > 0){
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(1);
        }
        if(gamepad1.right_trigger <= 0){
            intake.setPower(0);
        }

        if(gamepad2.rightBumperWasPressed()){
            motorPower1+=.05;
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake3.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake1.setPower(motorPower1);
            outtake2.setPower(motorPower1);
            outtake3.setPower(motorPower1);

        }
        if(gamepad2.leftBumperWasPressed()){
            motorPower1-=.05;
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake3.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake1.setPower(motorPower1);
            outtake2.setPower(motorPower1);
            outtake3.setPower(motorPower1);
        }

        if(gamepad2.aWasPressed()){
            fanRotate.setPosition(currPosFan);
            count=1;
        }

        if(gamepad1.yWasPressed()){
            automatedDrive = true;
            follower.followPath(pathChain1.get());

        }
        if(gamepad1.xWasPressed()){
            automatedDrive = true;
            follower.followPath(pathChain2.get());

        }
        if (automatedDrive && !follower.isBusy()) {
            automatedDrive = false;
            follower.startTeleopDrive();

        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("Motor Power: ", motorPower1);
    }
}