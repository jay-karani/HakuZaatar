package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsys;
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsys;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsys;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsys;

public class Robot{
    //subsystems
    private HoodSubsys hood;
    private IntakeSubsys intake;
    private ShooterSubsys shooter;
    private StopperSubsys stopper;
    private TurretSubsys turret;
    public LocalizerSubsys localizer;
    private Pose realGoal;

    public InterpLUT sotm;

    public Robot(HardwareMap hwMap, Follower follower){
        hood = new HoodSubsys(hwMap);
        intake = new IntakeSubsys(hwMap);
        shooter = new ShooterSubsys(hwMap);
        stopper = new StopperSubsys(hwMap);
        turret = new TurretSubsys(hwMap);
        localizer = new LocalizerSubsys(hwMap, follower);
        realGoal = RobotConstants.redGoal;
        localizer.initLocalizer();

        sotm = new InterpLUT();
        sotm.add(49.99, 2.36);
        sotm.add(81.84, 2.35);
        sotm.add(103.9, 2.82);
        sotm.add(130.33, 3.18);
        sotm.add(142.85, 0.9);
        sotm.createLUT();
    }

    public void backgroundUpdate(){
        localizer.updateLocalization();
        turret.turretTrack(RobotConstants.lastPedroPose);
        //doSOTM();
        if (RobotConstants.shooterOn){
            shooter.runLUT(localizer.getDistance());
            hood.runLUT(localizer.getDistance());
        }
    }

    public void runIntake(){
        intake.runIntake(1);
        stopper.stopperClose();
    }
    public void shootIntake(){
        intake.runIntake(1);
        stopper.stopperOpen();
    }
    public void idleIntake(){
        intake.idle();
    }

    public void setShooterOn(boolean shooterState){
        RobotConstants.shooterOn = shooterState;
    }

    public void doSOTM(){
        Pose robotPose = localizer.getPedroPose();
        Vector velocity = localizer.getRobotVelocity();
        double distance = localizer.getDistance();

        double shotTime = sotm.get(distance);
        shotTime *= RobotConstants.shotTimeMult;
        double offsetX = velocity.getXComponent() * shotTime;
        double offsetY = velocity.getYComponent() * shotTime;
        Pose newGoal = new Pose(realGoal.getX() - offsetX, realGoal.getY() - offsetY);
        double dx = newGoal.getX() + robotPose.getX();
        double dy = newGoal.getY() + robotPose.getY();

        distance = Math.hypot(dx, dy);
        shooter.runLUT(distance);
        hood.runLUT(distance);

        double turretAngle = Math.atan2(dy, dx) - robotPose.getHeading();
        turret.toAngle(Math.toDegrees(turretAngle));
    }

    public void offsetTurret(double change){
        turret.changeOffset(change);
    }
    public void offsetShooter(double change){
        shooter.changeOffset(change);
    }
}
