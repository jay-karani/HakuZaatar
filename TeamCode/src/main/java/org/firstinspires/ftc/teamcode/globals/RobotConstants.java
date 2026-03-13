package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class RobotConstants {

    public enum RobotTeam{
        RED, BLUE
    }
    public static RobotTeam robotTeam = RobotTeam.RED;

    //goal poses
    public static Pose redGoal = new Pose(144 ,144);
    public static Pose blueGoal = new Pose(0, 144);

    public static double shotTimeMult = 1;

    //hood
    public static final String hood_name = "hood_servo";
    public static double hoodMax = 0.99, hoodMin = 0;

    //swerve
    public static final double analogMax = 3.1865;
    public static final double driveServoCaching = 0.0001;

    public static PIDFCoefficients flPIDF = new PIDFCoefficients(0.15, 0, 0.002, 0.07);
    public static double flOffset = 1.92;

    public static double blOffset = 0.05;

    public static double brOffset = 2.7;

    public static double frOffset = 0.2;

    //pose converters
    public static Pose2D pedroToFTC(Pose pose) {
        double x = 72 - pose.getY();
        double y = pose.getX() - 72;
        double heading = pose.getHeading() + 90;
        // normalize
        heading = (heading % 360 + 360) % 360;
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }
    public static Pose ftcToPedro(Pose2D pose2d) {
        double x = pose2d.getY(DistanceUnit.INCH) + 72;
        double y = 72 - pose2d.getX(DistanceUnit.INCH);
        double heading = pose2d.getHeading(AngleUnit.DEGREES) - 90;
        // normalize to [0, 360)
        heading = (heading % 360 + 360) % 360;
        return new Pose(x, y, heading);
    }


    //intake
    public static final String leftIntake = "left_intake";
    public static final String rightIntake = "right_intake";
    public static final boolean rightIntakeReversed = false;
    public static double intakeBoost = 0;

    //turret
    public static final String leftTurret = "left_turret";
    public static final String rightTurret = "right_turret";
    public static final double turretZero = 0.5;
    public static double turretServoRange = 280;
    public static double turretMaxLeft = -180;
    public static double turretMaxRight = 180;

    //shooter
    public static final String leftShooter = "left_shooter";
    public static final String rightShooter=  "right_shooter";
    public static final boolean leftShooterReversed = true;
    public static double shooterP = 0.001, shooterI = 0, shooterD = 0, shooterF = 0.0004;

    //stopper
    public static final String stopperName = "stopper";
    public static double stopperMin = 0.41;
    public static double stopperMax = 0.28;

    public static Pose startingPosition = new Pose(0, 0, 0);
}
