package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class StopperSubsys extends SubsystemBase {
    private ServoEx stopperServo;
    private boolean stopperClosed = false;

    public StopperSubsys(final HardwareMap hwMap){
        stopperServo = new ServoEx(hwMap, RobotConstants.stopperName);
    }

    public void stopperOpen(){
        stopperServo.set(RobotConstants.stopperMin);
    }
    public void stopperClose(){
        stopperServo.set(RobotConstants.stopperMax);
    }

}
