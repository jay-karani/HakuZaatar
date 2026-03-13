package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsys;
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsys;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsys;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsys;

@Autonomous
public class RedClose extends CommandOpMode {
    private Follower follower;
    private TelemetryData telemetryData = new TelemetryData(telemetry);
    private LocalizerSubsys localizer;

    private ShooterSubsys shooter;
    private HoodSubsys hood;
    private StopperSubsys stopper;
    private TurretSubsys turret;
    private IntakeSubsys intake;
    public double distanceToGoal = 0;
    public boolean shooterOn = false;

    public final Pose startPose = new Pose(111, 135, Math.toRadians(270));
    public final Pose shootPose = new Pose(101, 104, Math.toRadians(0));
    public final Pose secondLine = new Pose(98, 60, Math.toRadians(0));
    public final Pose pickSecondLine = new Pose(131, 60, Math.toRadians(0));
    public final Pose backGate = new Pose(110, 78, Math.toRadians(0));
    public final Pose emptyGate = new Pose(122, 78, Math.toRadians(0));
    public final Pose gate = new Pose(147, 44, Math.toRadians(45));

    private PathChain shootPreload, goToSecondLine, goPickSecondLine, goEmptyGate, shootSecondLine, goOpenGate, shootGate;

    public void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        goToSecondLine = follower.pathBuilder().addPath(new BezierLine(shootPose, secondLine))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondLine.getHeading())
                .build();

        goPickSecondLine = follower.pathBuilder().addPath(new BezierLine(secondLine, pickSecondLine))
                .setLinearHeadingInterpolation(secondLine.getHeading(), pickSecondLine.getHeading())
                .build();

        goEmptyGate = follower.pathBuilder().addPath(new BezierLine(pickSecondLine, backGate))
                .setLinearHeadingInterpolation(pickSecondLine.getHeading(), backGate.getHeading())
                .addPath(new BezierLine(backGate, emptyGate))
                .setLinearHeadingInterpolation(backGate.getHeading(), emptyGate.getHeading())
                .build();

        shootSecondLine = follower.pathBuilder().addPath(new BezierLine(pickSecondLine, shootPose))
                .setLinearHeadingInterpolation(emptyGate.getHeading(), shootPose.getHeading())
                .build();


        goOpenGate = follower.pathBuilder().addPath(new BezierLine(shootPose, gate))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gate.getHeading())
                .build();




    }

    private RunCommand updatePoses = new RunCommand(() -> localizer.updateLocalization());
    private RunCommand updateDistance = new RunCommand(() -> distanceToGoal = localizer.getDistance());
    private RunCommand trackTurret = new RunCommand(() -> turret.turretTrack(localizer.getPedroPose()));
    private RunCommand runHood = new RunCommand(() -> hood.runLUT(distanceToGoal));
    private RunCommand updateRobotPose = new RunCommand(() -> RobotConstants.startingPosition = follower.getPose());


    private RunCommand runShooter = new RunCommand(() -> {
        if (shooterOn) {
            shooter.runLUT(distanceToGoal);
            stopper.stopperOpen();
        } else {
            stopper.stopperClose();
        }
    });

    @Override
    public void initialize() {
        super.reset();
        RobotConstants.startingPosition = new Pose(0, 0, 0);
        RobotConstants.robotTeam = RobotConstants.robotTeam;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        localizer = new LocalizerSubsys(hardwareMap, follower);
        localizer.initLocalizer();
        localizer.updateLocalization();

        shooter = new ShooterSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);
        stopper = new StopperSubsys(hardwareMap);
        turret = new TurretSubsys(hardwareMap);
        intake = new IntakeSubsys(hardwareMap);
        stopper.stopperClose();


        buildPaths();

        schedule(
                new RunCommand(() -> follower.update()), updatePoses, updateDistance,
                trackTurret, runHood, runShooter, updateRobotPose,
                new SequentialCommandGroup(
                        new InstantCommand(() -> shooterOn = !shooterOn),
                        new FollowPathCommand(follower, shootPreload, true),
                        new WaitCommand(500), new InstantCommand(() -> intake.runIntake(0.8)), new WaitCommand(1500),
                        new InstantCommand(() -> intake.runIntake(0.3)),
                        new InstantCommand(() -> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, goToSecondLine), new InstantCommand(() -> intake.runIntake(1)),
                        new FollowPathCommand(follower, goPickSecondLine),
                        new WaitCommand(100), new InstantCommand(() -> intake.runIntake(0.3)),
                        new FollowPathCommand(follower, goEmptyGate),

                        new ParallelCommandGroup(new FollowPathCommand(follower, shootSecondLine), new SequentialCommandGroup(
                                new WaitCommand(100), new InstantCommand(() -> shooterOn = !shooterOn)
                        )),
                        new WaitCommand(650),
                        new InstantCommand(() -> intake.runIntake(0.8)),
                        new WaitCommand(1500),
                        new InstantCommand(() -> intake.runIntake(0.3)),
                        new InstantCommand(() -> shooterOn = !shooterOn),


                        new FollowPathCommand(follower, goOpenGate), new InstantCommand(() -> intake.runIntake(1))

                )
        );


    }

    @Override
    public void run() {
        super.run();

        telemetryData.addData("ftcX", localizer.getFtcPose().getX(DistanceUnit.INCH));
        telemetryData.addData("ftcY", localizer.getFtcPose().getY(DistanceUnit.INCH));
        telemetryData.addData("distanceReported", localizer.getDistance());
        telemetryData.addData("actual distance", distanceToGoal);
        telemetryData.update();

    }

}
