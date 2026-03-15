package org.firstinspires.ftc.teamcode.opModes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
public class red18Auto extends CommandOpMode {
    public Follower follower;
    public Robot robot;

    public final Pose startPose = RobotConstants.redFrontAuto;
    public final Pose shootPose = new Pose(86, 83, Math.toRadians(0));
    public final Pose secondLine = new Pose(104, 59, Math.toRadians(0));
    public final Pose intakeSecondLine = new Pose(120, 59, Math.toRadians(0));
    public final Pose alignGate = new Pose(122, 69, Math.toRadians(0));
    public final Pose gateCurve = new Pose(101, 69);
    public final Pose intakeGate = new Pose(134, 57, Math.toRadians(45));
    public final Pose firstLine = new Pose(120, 83, Math.toRadians(0));
    public final Pose park = new Pose(86, 67, Math.toRadians(0));

    private PathChain shootPreload, goToSecondLine, pickSecondLine, shootSecondLine,
            openGate, gateCycle, shootGate, intakeFirst, backFirst, goPark;

    public void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        goToSecondLine = follower.pathBuilder().addPath(new BezierLine(shootPose, secondLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondLine.getHeading())
                .build();

        pickSecondLine = follower.pathBuilder().addPath(new BezierLine(secondLine, intakeSecondLine))
                .setLinearHeadingInterpolation(secondLine.getHeading(), intakeSecondLine.getHeading())
                .build();


        shootSecondLine = follower.pathBuilder().addPath(new BezierLine(intakeSecondLine, shootPose))
                .setLinearHeadingInterpolation(intakeSecondLine.getHeading(), shootPose.getHeading())
                .build();


        openGate = follower.pathBuilder().addPath(new BezierCurve(shootPose, gateCurve, alignGate))
                .setLinearHeadingInterpolation(shootPose.getHeading(), alignGate.getHeading())
                .build();

        gateCycle = follower.pathBuilder().addPath(new BezierLine(alignGate, intakeGate))
                .setLinearHeadingInterpolation(alignGate.getHeading(), intakeGate.getHeading())
                .build();

        shootGate = follower.pathBuilder().addPath(new BezierLine(intakeGate, shootPose))
                .setLinearHeadingInterpolation(intakeGate.getHeading(), shootPose.getHeading())
                .build();

        intakeFirst = follower.pathBuilder().addPath(new BezierLine(shootPose, firstLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstLine.getHeading())
                .build();

        backFirst = follower.pathBuilder().addPath(new BezierLine(firstLine, shootPose))
                .setLinearHeadingInterpolation(firstLine.getHeading(), shootPose.getHeading())
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
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(false)),

                        new FollowPathCommand(follower, goToSecondLine),
                        new InstantCommand(()-> robot.runIntake()),
                        new FollowPathCommand(follower, pickSecondLine),
                        new InstantCommand(() -> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(true)),
                        new FollowPathCommand(follower, shootSecondLine),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(false)),

                        //gate intake
                        new FollowPathCommand(follower, openGate),
                        new InstantCommand(()-> robot.runIntake()),
                        new FollowPathCommand(follower, gateCycle),
                        new WaitCommand(750),
                        new InstantCommand(()-> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(true)),
                        new FollowPathCommand(follower, shootGate),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(false)),

                        new FollowPathCommand(follower, openGate),
                        new InstantCommand(()-> robot.runIntake()),
                        new FollowPathCommand(follower, gateCycle),
                        new WaitCommand(750),
                        new InstantCommand(()-> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(true)),
                        new FollowPathCommand(follower, shootGate),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(false)),

                        new FollowPathCommand(follower, openGate),
                        new InstantCommand(()-> robot.runIntake()),
                        new FollowPathCommand(follower, gateCycle),
                        new WaitCommand(750),
                        new InstantCommand(()-> robot.idleIntake()),
                        new InstantCommand(()-> robot.setShooterOn(true)),
                        new FollowPathCommand(follower, shootGate),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.runIntake()),
                        new InstantCommand(()-> robot.setShooterOn(false)),

                        //firstline
                        new FollowPathCommand(follower, intakeFirst),
                        new InstantCommand(()-> robot.setShooterOn(true)),
                        new InstantCommand(() -> robot.idleIntake()),
                        new FollowPathCommand(follower, backFirst),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.shootIntake()),
                        new WaitCommand(500),
                        new InstantCommand(()-> robot.setShooterOn(false)),
                        new InstantCommand(()-> RobotConstants.useBoost = false),
                        new FollowPathCommand(follower, goPark, true)
                )
        );
    }

    @Override
    public void run(){
        super.run();
    }
}
