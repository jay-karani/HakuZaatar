package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-114.816039)
            .lateralZeroPowerAcceleration(-64.5960767)
            .translationalPIDFCoefficients(new PIDFCoefficients(.07, 0, 0.008, 0.05));

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-102.5/25.4)
            .strafePodX(107.5/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .xVelocity(63.737230)
            .yVelocity(68.883364)
            .maxPower(0.8);


    private static CoaxialPod leftFront(HardwareMap hardwareMap){
         CoaxialPod pod = new CoaxialPod(hardwareMap, "fl_motor", "fl_servo", "fl_analog", RobotConstants.swervePIDF,
                 DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, RobotConstants.flOffset, new Pose(139, 162),
                 0, RobotConstants.analogMax, false);
         pod.setServoCachingThreshold(RobotConstants.driveServoCaching); return pod;
    }
    private static CoaxialPod rightFront(HardwareMap hardwareMap){
        CoaxialPod pod = new CoaxialPod(hardwareMap, "fr_motor", "fr_servo", "fr_analog", RobotConstants.swervePIDF,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, RobotConstants.frOffset, new Pose(139, -162),
                0, RobotConstants.analogMax, false);
        pod.setServoCachingThreshold(RobotConstants.driveServoCaching); return pod;
    }
    private static CoaxialPod leftBack(HardwareMap hardwareMap){
        CoaxialPod pod = new CoaxialPod(hardwareMap, "bl_motor", "bl_servo", "bl_analog", RobotConstants.swervePIDF,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, RobotConstants.blOffset, new Pose(-139, 162),
                0, RobotConstants.analogMax, false);
        pod.setServoCachingThreshold(RobotConstants.driveServoCaching); return pod;
    }
    private static CoaxialPod rightBack(HardwareMap hardwareMap){
        CoaxialPod pod = new CoaxialPod(hardwareMap, "br_motor", "br_servo", "br_analog", RobotConstants.swervePIDF,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, RobotConstants.brOffset, new Pose(-139, -162),
                0, RobotConstants.analogMax, false);
        pod.setServoCachingThreshold(RobotConstants.driveServoCaching); return pod;
    }


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap), leftBack(hardwareMap), rightBack(hardwareMap))
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}