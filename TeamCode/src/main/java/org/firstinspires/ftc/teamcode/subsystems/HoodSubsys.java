package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class HoodSubsys extends SubsystemBase {

    private ServoEx hoodServo;
    private double hoodPos = 0.0;
    private LUT<Double, Double> hoodLUT;

    public HoodSubsys(final HardwareMap hwMap){
        hoodServo = new ServoEx(hwMap, RobotConstants.hood_name);
    }

    public void hoodTo(double position){
        double clampedPos = MathUtils.clamp(position, RobotConstants.hoodMin, RobotConstants.hoodMax);
        this.hoodPos = clampedPos;
        hoodServo.set(this.hoodPos);

        hoodLUT = new LUT<Double, Double>(){{
            add(43.7, 0.0);
            add(61.6, 0.5);
            add(77.3, 0.65);
            add(97.8, 0.8);
            add(122.75, 0.9);
            add(138.2, 1.0);
            add(160.3, 1.0);
        }};

    }

    public double getPos(){
        return this.hoodPos;
    }

    public void runLUT(double distance){
        double target = hoodLUT.getClosest(distance);
        double clampedPos = MathUtils.clamp(target, RobotConstants.hoodMin, RobotConstants.hoodMax);
        this.hoodPos = clampedPos;
        hoodServo.set(this.hoodPos);
    }
}
