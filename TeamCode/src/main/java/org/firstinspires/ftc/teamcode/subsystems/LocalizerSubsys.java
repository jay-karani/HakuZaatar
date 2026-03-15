package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class LocalizerSubsys extends SubsystemBase {

    private Follower follower;
    //private Pose2D ftcPose;
    //private Pose pedroPose;
    private LLResult result;
    private Pose goalPose;
    private double distanceToGoal;
    //private Pose3D limelightPose;
    //private Vector robotVelocity;

    private Limelight3A limelight;

    public LocalizerSubsys(final HardwareMap hwMap, Follower follower){
        this.follower = follower;
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void initLocalizer(){
        follower.update();
        if (RobotConstants.robotTeam == RobotConstants.RobotTeam.RED){
            limelight.pipelineSwitch(0);
            goalPose = RobotConstants.redGoal;
        } else {
            limelight.pipelineSwitch(1);
            goalPose = RobotConstants.blueGoal;
        }
    }

    public void updateLocalization(){
        follower.update();
        RobotConstants.lastPedroPose = follower.getPose();
        RobotConstants.lastFTCPose = RobotConstants.pedroToFTC(RobotConstants.lastPedroPose);

        this.result = limelight.getLatestResult();
        limelight.updateRobotOrientation(RobotConstants.lastFTCPose.getHeading(AngleUnit.DEGREES) - RobotConstants.turretAngle);
        if (result != null && result.isValid()) {
            RobotConstants.lastLimelightPose = result.getBotpose_MT2();
        }
        this.distanceToGoal = goalPose.distanceFrom(RobotConstants.lastPedroPose);
        RobotConstants.lastVelocity = follower.getVelocity();
    }

    public double getDistance(){
        return this.distanceToGoal;
    }

    public Pose2D getFtcPose(){
        return RobotConstants.lastFTCPose;
    }
    public Vector getRobotVelocity(){
        return RobotConstants.lastVelocity;
    }

    public Pose getPedroPose(){
        return RobotConstants.lastPedroPose;
    }

    public Pose getGoalPose(){
        return this.goalPose;
    }

    public void relocalize(){
        double x = RobotConstants.lastLimelightPose.getPosition().x;
        double y = RobotConstants.lastLimelightPose.getPosition().y;
        RobotConstants.lastFTCPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, RobotConstants.lastFTCPose.getHeading(AngleUnit.DEGREES));
    }
}
