package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class TurretSubsys extends SubsystemBase {

    private ServoEx leftTurret, rightTurret;
    private Pose goalPose;
    public double offset = 0;

    public TurretSubsys(final HardwareMap hwMap){
        leftTurret = new ServoEx(hwMap, RobotConstants.leftTurret);
        rightTurret = new ServoEx(hwMap, RobotConstants.rightTurret);
        if (RobotConstants.robotTeam == RobotConstants.RobotTeam.RED){
            this.goalPose = RobotConstants.redGoal;
        } else {this.goalPose = RobotConstants.blueGoal;}
    }

    public void toAngle(double angle){
        double gearRatio = (angle + offset) * 160.0 / 38.0 * 15.0 / 80.0;
        double servoTarget = RobotConstants.turretZero + (gearRatio / RobotConstants.turretServoRange);
        RobotConstants.turretAngle = angle;
        leftTurret.set(servoTarget);
        rightTurret.set(servoTarget);
    }

    public void turretTrack(Pose robotPose) {

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double fieldAngle = Math.atan2(dy, dx);

        // Flip sign because turret + is clockwise
        double turretAngleRad = robotPose.getHeading() - fieldAngle;

        double turretAngleDeg = Math.toDegrees(turretAngleRad);

        turretAngleDeg = ((turretAngleDeg + 180) % 360) - 180;

        toAngle(turretAngleDeg);
    }

    public void changeOffset(double change){
        this.offset += change;
    }

}
