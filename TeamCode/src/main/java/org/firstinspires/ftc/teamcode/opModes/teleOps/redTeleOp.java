package org.firstinspires.ftc.teamcode.opModes.teleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class redTeleOp extends OpMode {

    public Robot robot;
    public Follower follower;

    @Override
    public void init() {
        RobotConstants.robotTeam = RobotConstants.RobotTeam.RED;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(RobotConstants.lastPedroPose);
        follower.startTeleOpDrive(true);
        follower.update();
        robot = new Robot(hardwareMap, follower);
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false, Math.toRadians(0));
        robot.backgroundUpdate();

        if (gamepad1.rightBumperWasPressed()){ //toggle shooter
            robot.setShooterOn(!RobotConstants.shooterOn);
        }

        if (gamepad1.right_trigger > 0.2){
            robot.runIntake();
        } else if (gamepad1.left_trigger > 0.2) {
            robot.shootIntake();
        } else {
            robot.idleIntake();
        }

        if (gamepad1.leftBumperWasPressed()){
            RobotConstants.useBoost = !RobotConstants.useBoost;
        }

        if (gamepad1.dpadLeftWasPressed()){
            robot.offsetTurret(-5);
        } else if (gamepad1.dpadRightWasPressed()){
            robot.offsetTurret(5);
        }

        if (gamepad1.dpadUpWasPressed()){
            robot.offsetShooter(20);
        } else if (gamepad1.dpadDownWasPressed()){
            robot.offsetShooter(-20);
        }

        if (gamepad1.optionsWasPressed()){
            robot.localizer.relocalize();
        }
    }
}
