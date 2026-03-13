package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class Tester2 extends OpMode {
    public Robot robot;
    public Follower follower;
    public Pose startPose = new Pose(72, 72, Math.toRadians(90));

    @Override
    public void init() {
        RobotConstants.robotTeam = RobotConstants.RobotTeam.RED;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleOpDrive(true);
        follower.update();
        robot = new Robot(hardwareMap, follower);
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        robot.backgroundUpdate();
        if (gamepad1.right_trigger > 0.2){
            robot.runIntake();
        } else if (gamepad1.left_trigger > 0.2){
            robot.shootIntake();
        }
        telemetry.addData("distance", robot.localizer.getDistance());
        telemetry.update();
    }
}
