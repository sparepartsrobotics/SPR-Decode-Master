package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.67)
            .forwardZeroPowerAcceleration(-49.26596618615037)
            .lateralZeroPowerAcceleration(-55.04185882855684)
            .translationalPIDFCoefficients(new PIDFCoefficients(.05,0,.005,.005))
            .translationalPIDFSwitch(4)
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,.02,.0025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.0025,0,.000005,.2,.00025))
            .drivePIDFSwitch(20)
            .centripetalScaling(.0005);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            //leftFront: 0
            //rightFront: 1
            //leftRear: 2
            //rightRear: 3
            .maxPower(.5)
            .xVelocity(90.54726031607169)
            .yVelocity(53.77539485081231)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            //leftEncoder: 2
            //rightEncoder: 0
            //strafeEncoder: 1
            .forwardTicksToInches(.0030374001108549537)
            .strafeTicksToInches(.00291391526774855)
            .turnTicksToInches((.0029602123684673355 + .0029534658301083685)/2)
            .leftPodY(-4.25)
            .rightPodY(4)
            .strafePodX(-7)

            .leftEncoder_HardwareMapName("rightFront")
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("leftRear")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);
}
