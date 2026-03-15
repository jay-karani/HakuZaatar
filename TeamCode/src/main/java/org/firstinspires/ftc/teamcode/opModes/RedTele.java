package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

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
public class RedTele extends CommandOpMode {

    private Follower follower;
    private LocalizerSubsys localizer;

    private ShooterSubsys shooter;
    private HoodSubsys hood;
    private StopperSubsys stopper;
    private TurretSubsys turret;
    private IntakeSubsys intake;

    private TelemetryData telemetryData;

    private boolean shooterOn = false;
    private boolean intakeFast = false;
    private double distanceToGoal = 0;

    // edge detection
    private boolean prevX = false;

    @Override
    public void initialize() {

        super.reset();

        RobotConstants.robotTeam = RobotConstants.RobotTeam.RED;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(104, 76, Math.toRadians(90)));
        follower.startTeleOpDrive();
        follower.update();

        localizer = new LocalizerSubsys(hardwareMap, follower);
        localizer.initLocalizer();
        localizer.updateLocalization();

        intake = new IntakeSubsys(hardwareMap);
        turret = new TurretSubsys(hardwareMap);
        stopper = new StopperSubsys(hardwareMap);
        shooter = new ShooterSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);

        stopper.stopperClose();

        telemetryData = new TelemetryData(telemetry);

        schedule(

                // follower update
                new RunCommand(() -> follower.update()),

                // localization update
                new RunCommand(() -> localizer.updateLocalization()),

                // update distance
                new RunCommand(() -> distanceToGoal = localizer.getDistance()),

                // turret tracking
                new RunCommand(() -> {
                    Pose pose = follower.getPose();
                    if (pose != null) {
                        turret.turretTrack(pose);
                    }
                }),

                // hood auto LUT
                new RunCommand(() -> hood.runLUT(distanceToGoal)),

                // shooter + stopper control based on toggle
                new RunCommand(() -> {
                    // handle single toggle
                    boolean currX = gamepad1.x;
                    if (currX && !prevX) {
                        shooterOn = !shooterOn;
                    }
                    prevX = currX;

                    if (shooterOn) {
                        shooter.runLUT(distanceToGoal);
                        stopper.stopperOpen();
                    } else {
                        stopper.stopperClose();
                    }
                }),

                // idle intake
                new RunCommand(() -> {
                    if (!intakeFast) {
                        intake.runIntake(0.3);
                    }
                }),

                // DRIVER CONTROLS
                new RunCommand(() -> {

                    // drive
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x,
                            false
                    );

                    // fast intake
                    intakeFast = gamepad1.right_bumper;
                    if (intakeFast) {
                        intake.runIntake(0.8);
                    }

                    // hood manual adjust
                    if (gamepad1.dpad_up) hood.hoodTo(hood.getPos() + 0.01);
                    if (gamepad1.dpad_down) hood.hoodTo(hood.getPos() - 0.01);

                    // telemetry
                    Pose pose = follower.getPose();
                    if (pose != null) {
                        telemetryData.addData("x", pose.getX());
                        telemetryData.addData("y", pose.getY());
                        telemetryData.addData("theta", pose.getHeading());
                    }

                    telemetryData.addData("distance", distanceToGoal);
                    telemetryData.addData("shooterOn", shooterOn);
                    telemetryData.addData("intakeFast", intakeFast);
                    telemetryData.addData("hoodPos", hood.getPos());

                    telemetryData.update();
                })
        );
    }
}