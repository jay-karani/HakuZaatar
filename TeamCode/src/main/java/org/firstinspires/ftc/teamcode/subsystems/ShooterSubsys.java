package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class ShooterSubsys extends SubsystemBase {
    private Motor leftLeader, rightFollower;
    private PIDFController pidf = new PIDFController(RobotConstants.shooterP, RobotConstants.shooterI,
            RobotConstants.shooterD, RobotConstants.shooterF);
    InterpLUT shooterLUT;

    public ShooterSubsys(final HardwareMap hwMap){
        leftLeader =  new Motor(hwMap, RobotConstants.leftShooter).setInverted(RobotConstants.leftShooterReversed)
                .setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftLeader.setRunMode(Motor.RunMode.RawPower);
        rightFollower = new Motor(hwMap, RobotConstants.rightShooter).setInverted(!RobotConstants.leftShooterReversed)
                .setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        shooterLUT = new InterpLUT();
        shooterLUT.add(43.7, 1300);
        shooterLUT.add(61.6, 1500);
        shooterLUT.add(77.3, 1600);
        shooterLUT.add(97.8, 1750);
        shooterLUT.add(122.75, 1900);
        shooterLUT.add(138.2, 2010);
        shooterLUT.add(160.3, 2065);
        shooterLUT.createLUT();
    }

    public void setPower(double power){
        double vel = leftLeader.getCorrectedVelocity();
        double output = pidf.calculate(vel, power);
        leftLeader.set(output);
        rightFollower.set(output);
    }

    public void runLUT(double distance){
        double vel = leftLeader.getCorrectedVelocity();
        double target = shooterLUT.get(distance);
        double output = pidf.calculate(vel, target);
        leftLeader.set(output);
        rightFollower.set(output);
    }

    public double getVelocity(){
        return leftLeader.getCorrectedVelocity();
    }

    public void updatePIDF(){
        pidf.setPIDF(RobotConstants.shooterP, RobotConstants.shooterI,
                RobotConstants.shooterD, RobotConstants.shooterF);
    }
    public void idle(){
        leftLeader.set(0);
        rightFollower.set(0);
    }
}
