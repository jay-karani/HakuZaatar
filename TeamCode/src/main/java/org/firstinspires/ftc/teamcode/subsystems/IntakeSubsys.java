package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class IntakeSubsys extends SubsystemBase {

    private Motor rightLeader, leftFollower;

    public IntakeSubsys(final HardwareMap hwMap){
        rightLeader = new Motor(hwMap, RobotConstants.rightIntake, Motor.GoBILDA.BARE)
                .setInverted(RobotConstants.rightIntakeReversed).setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLeader.setRunMode(Motor.RunMode.RawPower);
        leftFollower = new Motor(hwMap, RobotConstants.leftIntake, Motor.GoBILDA.BARE)
                .setInverted(!RobotConstants.rightIntakeReversed).setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFollower.setRunMode(Motor.RunMode.RawPower);
    }

    public void runIntake(double power){
        rightLeader.set(power);
        leftFollower.set(power);
    }

    public void idle(){
        rightLeader.set(RobotConstants.intakeBoost);
        leftFollower.set(RobotConstants.intakeBoost);
    }
}
