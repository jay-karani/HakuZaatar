package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class LocalizerSubsys extends SubsystemBase {

    private Follower follower;
    private Pose2D ftcPose;
    private Pose pedroPose;
    private LLResult result;
    private Pose goalPose;
    private double distanceToGoal;
    private Pose3D limelightPose;
    private Vector robotVelocity;

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
        this.pedroPose = follower.getPose();
        this.ftcPose = RobotConstants.pedroToFTC(this.pedroPose);

        this.result = limelight.getLatestResult();
        limelight.updateRobotOrientation(this.ftcPose.getHeading(AngleUnit.DEGREES));
        if (result != null && result.isValid()) {
            this.limelightPose = result.getBotpose_MT2();
        }
        this.distanceToGoal = goalPose.distanceFrom(pedroPose);
        this.robotVelocity = follower.getVelocity();
    }

    public double getDistance(){
        return this.distanceToGoal;
    }

    public Pose getPedroPose() {return this.pedroPose;}

    public Pose2D getFtcPose(){
        return this.ftcPose;
    }
    public Vector getRobotVelocity(){
        return this.robotVelocity;
    }
}
