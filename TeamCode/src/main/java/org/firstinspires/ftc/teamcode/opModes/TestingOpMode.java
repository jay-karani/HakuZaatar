package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsys;
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsys;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsys;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsys;

@Disabled
@Configurable
@TeleOp
public class TestingOpMode extends OpMode {
    public IntakeSubsys intake;
    public TurretSubsys turret;
    public StopperSubsys stopper;
    public ShooterSubsys shooter;
    public HoodSubsys hood;
    public static double intakeSpeed;
    public static double turretTarget;
    public static double shooterTarget;
    public static double hoodPosition;
    public Follower follower;
    public LocalizerSubsys localizer;

    @Override
    public void init() {
        RobotConstants.robotTeam = RobotConstants.RobotTeam.RED;
        follower = Constants.createFollower(hardwareMap);
        localizer = new LocalizerSubsys(hardwareMap, follower);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        intake = new IntakeSubsys(hardwareMap);
        turret = new TurretSubsys(hardwareMap);
        stopper = new StopperSubsys(hardwareMap);
        shooter = new ShooterSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);
        localizer.initLocalizer();
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        stopper.stopperOpen();
        shooter.updatePIDF();
        turret.turretTrack(follower.getPose());
        intake.runIntake(gamepad1.right_trigger);
        shooter.setPower(shooterTarget);
        localizer.updateLocalization();
        telemetry.addData("targetVel", shooterTarget);
        telemetry.addData("currentVel", shooter.getVelocity());
        telemetry.addData("DISTANCE", localizer.getDistance());
        telemetry.addData("servo", hood.getPos());
        hood.hoodTo(hoodPosition);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("theta", follower.getPose().getHeading());
    }
}
