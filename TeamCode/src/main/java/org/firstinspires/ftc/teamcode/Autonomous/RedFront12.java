package org.firstinspires.ftc.teamcode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
public class RedFront12 extends CommandOpMode {
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
    public final Pose firstLine = new Pose(96, 88, Math.toRadians(0));
    public final Pose intakeFirst = new Pose(125, 88, Math.toRadians(0));
    public final Pose backGate = new Pose(112, 78, Math.toRadians(0));
    public final Pose emptyGate = new Pose(122, 78, Math.toRadians(0));
    public final Pose secondLine = new Pose(96, 58, Math.toRadians(0));
    public final Pose intakeSecond = new Pose(131, 58, Math.toRadians(0));
    public final Pose thirdLine = new Pose(96, 40, Math.toRadians(0));
    public final Pose intakeThird = new Pose(131, 40, Math.toRadians(0));
    public final Pose parkPose = new Pose(104, 76, Math.toRadians(90));

    private PathChain shootPreload, toFirstLine, pickFirstLine, goEmptyGate, shootFirstLine,
            driveToSecond, intakeSecondChain, shootSecondLine,
            driveToThird, intakeThirdChain,  shootThirdLine, park;

    public void buildPaths(){
        shootPreload = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        toFirstLine = follower.pathBuilder().addPath(new BezierLine(shootPose, firstLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstLine.getHeading())
                .build();
        pickFirstLine = follower.pathBuilder().addPath(new BezierLine(firstLine, intakeFirst))
                .setLinearHeadingInterpolation(firstLine.getHeading(), intakeFirst.getHeading())
                .build();

        goEmptyGate = follower.pathBuilder()
                .addPath(new BezierLine(intakeFirst, backGate))
                .setLinearHeadingInterpolation(intakeFirst.getHeading(), backGate.getHeading())
                .addPath(new BezierLine(backGate, emptyGate)) //new Pose(110, 83),
                .setLinearHeadingInterpolation(backGate.getHeading(), emptyGate.getHeading())
                .build();
        shootFirstLine = follower.pathBuilder().addPath(new BezierLine(emptyGate, shootPose))
                .setLinearHeadingInterpolation(intakeFirst.getHeading(), shootPose.getHeading())
                .build();
        driveToSecond = follower.pathBuilder().addPath(new BezierLine(shootPose, secondLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondLine.getHeading())
                .build();
        intakeSecondChain = follower.pathBuilder().addPath(new BezierLine(secondLine, intakeSecond))
                .setLinearHeadingInterpolation(secondLine.getHeading(), intakeSecond.getHeading())
                .build();

        shootSecondLine = follower.pathBuilder().addPath(new BezierCurve(intakeSecond, new Pose(97, 52), shootPose)) //
                .setLinearHeadingInterpolation(intakeSecond.getHeading(), shootPose.getHeading())
                .build();
        driveToThird = follower.pathBuilder().addPath(new BezierLine(shootPose, thirdLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdLine.getHeading())
                .build();
        intakeThirdChain = follower.pathBuilder().addPath(new BezierLine(thirdLine, intakeThird))
                .setLinearHeadingInterpolation(thirdLine.getHeading(), intakeThird.getHeading())
                .build();

        shootThirdLine = follower.pathBuilder().addPath(new BezierLine(intakeThird, shootPose)) // new Pose(100, 44)
                .setLinearHeadingInterpolation(intakeThird.getHeading(), shootPose.getHeading())
                .build();
        park = follower.pathBuilder().addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private RunCommand updatePoses = new RunCommand(() -> localizer.updateLocalization());
    private RunCommand updateDistance = new RunCommand(()-> distanceToGoal = localizer.getDistance());
    private RunCommand trackTurret = new RunCommand(() -> turret.turretTrack(localizer.getPedroPose()));
    private RunCommand runHood = new RunCommand(()-> hood.runLUT(distanceToGoal));
    private RunCommand updateRobotPose = new RunCommand(()-> RobotConstants.startingPosition = follower.getPose());
    public boolean intakeFast = false;

    private RunCommand idleIntake = new RunCommand(() -> {
        if (!intakeFast) {
            intake.runIntake(0.3);
        }
    });

    private RunCommand runShooter = new RunCommand(()->{
        if (shooterOn){
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
                new RunCommand(()-> follower.update()), updatePoses, updateDistance,
                trackTurret, runHood, runShooter, updateRobotPose, idleIntake,
                new SequentialCommandGroup(
                        new InstantCommand(() -> shooterOn = !shooterOn),
                        new FollowPathCommand(follower, shootPreload, true),

                        new WaitCommand(650),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),
                        new WaitCommand(1500),
                        new InstantCommand(() -> intakeFast = false),

                        new InstantCommand(() -> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, toFirstLine),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),
                        new FollowPathCommand(follower, pickFirstLine),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),

                        new WaitCommand(750),
                        new InstantCommand(() -> intakeFast = false),

                        new FollowPathCommand(follower, goEmptyGate, true),

                        new WaitCommand(650),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, shootFirstLine),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new InstantCommand(() -> shooterOn = !shooterOn)
                                )
                        ),

                        new WaitCommand(650),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),
                        new WaitCommand(1500),
                        new InstantCommand(() -> intakeFast = false),
                        new InstantCommand(() -> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, driveToSecond),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),
                        new FollowPathCommand(follower, intakeSecondChain),

                        new WaitCommand(200),
                        new InstantCommand(() -> intakeFast = false),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, shootSecondLine),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> shooterOn = !shooterOn)
                                )
                        ),

                        new WaitCommand(650),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),
                        new WaitCommand(1500),
                        new InstantCommand(() -> intakeFast = false),
                        new InstantCommand(() -> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, driveToThird),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),
                        new FollowPathCommand(follower, intakeThirdChain),

                        new WaitCommand(200),
                        new InstantCommand(() -> intakeFast = false),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, shootThirdLine),
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new InstantCommand(() -> shooterOn = !shooterOn)
                                )
                        ),

                        new WaitCommand(650),
                        new InstantCommand(() -> {
                            intakeFast = true;
                            intake.runIntake(0.8);
                        }),
                        new WaitCommand(1500),
                        new InstantCommand(() -> intakeFast = false),
                        new InstantCommand(() -> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, park, true)
                )
        );
    }

    @Override
    public void run(){
        super.run();

        telemetryData.addData("ftcX", localizer.getFtcPose().getX(DistanceUnit.INCH));
        telemetryData.addData("ftcY", localizer.getFtcPose().getY(DistanceUnit.INCH));
        telemetryData.addData("distanceReported", localizer.getDistance());
        telemetryData.addData("actual distance", distanceToGoal);
        telemetryData.update();
    }
}
