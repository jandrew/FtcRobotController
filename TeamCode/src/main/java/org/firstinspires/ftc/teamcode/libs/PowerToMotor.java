package org.firstinspires.ftc.teamcode.libs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.sqrt;

public class PowerToMotor {
    
    double motorSpeed = 0;
    double motorAngle = 0;
    boolean scaleToCircle = false;
    String motorName = "";

    public PowerToMotor(Double motorAngleSetting, String motorLabel){
        motorAngle = motorAngleSetting;
        motorName = motorLabel;
    }

    public void setPowerAngle(Double requestedPower, Double targetAngle){
        double angleDifference = targetAngle - motorAngle;
        motorSpeed = requestedPower * Math.sin(angleDifference);
        Double fullPower = sqrt(Math.pow(Math.tan(targetAngle), 2) + 1);
        if(scaleToCircle){
            motorSpeed = motorSpeed / fullPower;
        }
    }
//        opMode.telemetry.addData("Motor speed", " motor: (%s), speed: (%.2f)", motorName, motorSpeed);

    public double getPower(){
        return motorSpeed;
    }
}
