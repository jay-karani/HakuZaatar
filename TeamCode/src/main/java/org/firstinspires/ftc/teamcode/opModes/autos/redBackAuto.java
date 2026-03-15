package org.firstinspires.ftc.teamcode.opModes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class redBackAuto extends CommandOpMode {
    public Follower follower;
    public Robot robot;

    public final Pose startPose = RobotConstants.redBackAuto;
    public final Pose shootPose = new Pose(88, 18, Math.toRadians(0));
    public final Pose alignFirst = new Pose(103, 35, Math.toRadians(0));
    public final Pose firstLine = new Pose(122, 35, Math.toRadians(0));
    public final Pose square = new Pose(132, 9, Math.toRadians(0));
    public final Pose releasedPick = new Pose(134, 13, Math.toRadians(0));
    public final Pose park = new Pose(88, 27, Math.toRadians(0));

    private PathChain shootPreload, goToFirst, intakeFirst, shootFirst,
            intakeSquare, shootSquare, intakeReleased, shootReleased, goPark;

    public void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        goToFirst = follower.pathBuilder().addPath(new BezierLine(shootPose, alignFirst))
                .setLinearHeadingInterpolation(shootPose.getHeading(), alignFirst.getHeading())
                .build();

        intakeFirst = follower.pathBuilder().addPath(new BezierLine(alignFirst, firstLine))
                .setLinearHeadingInterpolation(alignFirst.getHeading(), firstLine.getHeading())
                .build();

        shootFirst = follower.pathBuilder().addPath(new BezierLine(firstLine, shootPose))
                .setLinearHeadingInterpolation(firstLine.getHeading(), shootPose.getHeading())
                .build();

        intakeSquare = follower.pathBuilder().addPath(new BezierLine(shootPose, square))
                .setLinearHeadingInterpolation(shootPose.getHeading(), square.getHeading())
                .build();

        shootSquare = follower.pathBuilder().addPath(new BezierLine(square, shootPose))
                .setLinearHeadingInterpolation(square.getHeading(), shootPose.getHeading())
                .build();

        intakeReleased = follower.pathBuilder().addPath(new BezierLine(shootPose, releasedPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), releasedPick.getHeading())
                .build();

        shootReleased = follower.pathBuilder().addPath(new BezierLine(releasedPick, shootPose))
                .setLinearHeadingInterpolation(releasedPick.getHeading(), shootPose.getHeading())
                .build();

        goPark = follower.pathBuilder().addPath(new BezierLine(shootPose, park))
                .setLinearHeadingInterpolation(shootPose.getHeading(), park.getHeading())
                .build();
    }

    public RunCommand backgroundTasks = new RunCommand(()-> robot.backgroundUpdate());

    @Override
    public void initialize() {
        super.reset();
        RobotConstants.lastPedroPose = new Pose(0, 0, Math.toRadians(0));
        RobotConstants.robotTeam = RobotConstants.RobotTeam.RED;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        robot = new Robot(hardwareMap, follower);

        buildPaths();

        schedule(
                backgroundTasks,
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::isStarted),
                        new InstantCommand(()-> robot.setShooterOn(true)),
                        new InstantCommand(()-> robot.idleIntake()),
                        new FollowPathCommand(follower, shootPreload),
                        new WaitCommand(700),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.idleIntake()),

                        new FollowPathCommand(follower, goToFirst),
                        new InstantCommand(()-> robot.runIntake()),
                        new FollowPathCommand(follower, intakeFirst),
                        new InstantCommand(()-> robot.idleIntake()),
                        new FollowPathCommand(follower, shootFirst),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.runIntake()),

                        new FollowPathCommand(follower, intakeSquare),
                        new WaitCommand(700),
                        new InstantCommand(()-> robot.idleIntake()),
                        new FollowPathCommand(follower, shootSquare),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.runIntake()),

                        new FollowPathCommand(follower, intakeSquare),
                        new WaitCommand(700),
                        new InstantCommand(()-> robot.idleIntake()),
                        new FollowPathCommand(follower, shootSquare),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.runIntake()),

                        new FollowPathCommand(follower, intakeReleased),
                        new InstantCommand(()-> robot.idleIntake()),
                        new FollowPathCommand(follower, shootReleased),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.runIntake()),

                        new FollowPathCommand(follower, intakeReleased),
                        new InstantCommand(()-> robot.idleIntake()),
                        new FollowPathCommand(follower, shootReleased),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(()-> RobotConstants.useBoost = false),
                        new InstantCommand(()-> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(false)),
                        new FollowPathCommand(follower, goPark, true)
                )
        );
    }

    @Override
    public void run(){
        super.run();
    }
}
