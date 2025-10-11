package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;

@Autonomous(name = "Sample Auto", group = "Examples")
public class SampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private HuskyLens huskyLens;


    private int pathState;
    private final Pose startPose = new Pose(88, 4, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose order1 = new Pose(108, 36, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose order2 = new Pose(108, 60, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose order3 = new Pose(108, 84, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose moveToAprilTag = new Pose(72, 72, Math.toRadians(90)); //
    private Path scorePreload;
    private PathChain moveToOrder1, moveToOrder2, moveToOrder3;
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, moveToAprilTag));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());
        moveToOrder1 = follower.pathBuilder()
                .addPath(new BezierLine(moveToAprilTag, order1))
                .setLinearHeadingInterpolation(moveToAprilTag.getHeading(), order1.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder2 = follower.pathBuilder()
                .addPath(new BezierLine(moveToAprilTag, order2))
                .setLinearHeadingInterpolation(moveToAprilTag.getHeading(), order2.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder3 = follower.pathBuilder()
                .addPath(new BezierLine(moveToAprilTag, order3))
                .setLinearHeadingInterpolation(moveToAprilTag.getHeading(), order3.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        switch (pathState) {
            case(0):
                follower.followPath(scorePreload);
                setPathState(1);
            case 1:
                if(blocks.length > 0){
                    for(int i = 0; i < blocks.length; ++i) {
                        if (blocks[i].id == 1) {
                            setPathState(2);
                        }
                        else if(blocks[i].id == 2){
                            setPathState(3);
                        }
                        else if(blocks[i].id == 3){
                            setPathState(4);
                        }
                        else{
                            setPathState(-1);
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

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(moveToOrder1,true);
                    setPathState(-1);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(moveToOrder2,true);
                    setPathState(-1);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(moveToOrder3,true);
                    setPathState(-1);
                }
                break;

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
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

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

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}