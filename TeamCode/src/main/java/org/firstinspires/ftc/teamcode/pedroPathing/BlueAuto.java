package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue Auto", group = "Examples")
public class BlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private HuskyLens huskyLens;
    private Servo fanRotate, cam;
    private DcMotorSimple outtake1, outtake2, outtake3, intake;
    private double currPosFan = .05, camPos = 1, currRelease=-.01;
    private double fanPos1 = .16, fanPos2 =  .27, fanPos3 =.38;
    private double upPos1 = .1, upPos2 =  .21, upPos3 =.32;
    private boolean x = true;
    private boolean x2 = true;
    private boolean launchStarted = false;
     private int id = -1;
    private int count = 1;
    private int count2 = 1;
    private int pathState;
    private final Pose startPose = new Pose(60, 6, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose detectPose = new Pose(67, 70, Math.toRadians(-90));
    private final Pose launchPose = new Pose(67, 81, Math.toRadians(319));
    private final Pose launchOrder = new Pose(64,36, Math.toRadians(180));
    private final Pose order3 = new Pose(50, 54.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order3s = new Pose(40,54.5,Math.toRadians(180));
    private final Pose order31 = new Pose(37, 54.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order32 = new Pose(33, 54.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private Path detect;
    private PathChain launch, moveToOrder3,moveToOrder31,moveToOrder32,moveToOrder3s;
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void buildPaths(){
        detect = new Path(new BezierLine(startPose, detectPose));
        detect.setConstantHeadingInterpolation(startPose.getHeading());
        launch = follower.pathBuilder()
                .addPath(new BezierLine(detectPose,launchPose))
                .setLinearHeadingInterpolation(detectPose.getHeading(), launchPose.getHeading())
                .build();
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose,launchOrder, order3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), order3s.getHeading())
                .build();
        moveToOrder3s = follower.pathBuilder()
                .addPath(new BezierLine(order3, order3s))
                .setLinearHeadingInterpolation(order3.getHeading(), order3s.getHeading())
                .build();
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder31 = follower.pathBuilder()
                .addPath(new BezierCurve(order3s, order31))
                .setLinearHeadingInterpolation(order3.getHeading(), order31.getHeading())
                .build();
        moveToOrder32 = follower.pathBuilder()
                .addPath(new BezierCurve(order31, order32))
                .setLinearHeadingInterpolation(order31.getHeading(), order32.getHeading())
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {


        switch (pathState) {
            case(0):
                outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake3.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setPower(.68);
                outtake2.setPower(.68);
                outtake3.setPower(.68);
                follower.followPath(detect);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    HuskyLens.Block[] blocks = huskyLens.blocks();
                    sleep(200);
                    if(blocks.length > 0){
                        telemetry.addData("Block count", blocks.length);
                        telemetry.addData("ID: ", blocks[0].id);
                        id = blocks[0].id;
                        if (blocks[0].id == 1) {
                            setPathState(2);
                            telemetry.update();
                        }
                        else if(blocks[0].id == 2){
                            setPathState(2);
                            telemetry.update();
                        }
                        else if(blocks[0].id == 3){
                            setPathState(2);
                            telemetry.update();
                        }
                        else{
                            setPathState(-1);
                            telemetry.update();
                        }
                    }
                }
                break;
            case 2:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if(!follower.isBusy() && !launchStarted) {
                    follower.followPath(launch,true);
                    launchStarted = true;
                }
                if(!follower.isBusy() && launchStarted){
                    launchArtifact();
                    runIntake();
                    setPathState(3);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                sleep(1000);
//
//                setPathState(-1);
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    follower.followPath(moveToOrder3);
                    fan1();
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    follower.followPath(moveToOrder3s);
                    fan2();
                    setPathState(5);
                }
                break;
            case 5:{
                if(!follower.isBusy()){
                    follower.followPath(moveToOrder31);
                    if(pathTimer.getElapsedTimeSeconds() > 0.3){ // wait a bit
                        fan3();
                        setPathState(6);
                    }
                    setPathState(6);
                }
                break;
            }
            case 6:{
                if(!follower.isBusy()){
                    follower.followPath(moveToOrder32);
                    stopIntake();
                    setPathState(-1);
                }
                break;
            }


        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        // Feedback to Driver Hub for debugging

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("ID: ", id);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        fanRotate = hardwareMap.get(Servo.class, "fanRotate");
        cam = hardwareMap.get(Servo.class, "cam");
        outtake1 = hardwareMap.get(DcMotorSimple.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorSimple.class, "outtake2");
        outtake3 = hardwareMap.get(DcMotorSimple.class, "outtake3");
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
        cam.setPosition(camPos);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        fanRotate.setPosition(upPos1);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    public void launchArtifact() throws InterruptedException {
        camUp();
        sleep(800);
        fanRotate.setPosition(upPos2);
        sleep(800);
        camUp();
        sleep(800);
        fanRotate.setPosition(upPos3);
        sleep(800);
        camUp();
        outtake1.setPower(0);
        outtake2.setPower(0);
        outtake3.setPower(0);
    }
    public void fan1(){
        fanRotate.setPosition(fanPos1);
    }
    public void fan2(){
        fanRotate.setPosition(fanPos2);
    }
    public void fan3(){
        fanRotate.setPosition(fanPos3);
    }

    public void camUp() throws InterruptedException {
        if (camPos == 1) {
            camPos = 0;
        } else if (camPos == 0) {
            camPos = 1;
        }
        cam.setPosition(camPos);
    }
    public void runIntake() throws InterruptedException {
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(1);
    }
    public void stopIntake(){
        intake.setPower(1);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    public Pose getFinalPose(){
        return order3;
    }
}