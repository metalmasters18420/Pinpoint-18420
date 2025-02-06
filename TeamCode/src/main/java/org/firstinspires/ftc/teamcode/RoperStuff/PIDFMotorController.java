package org.firstinspires.ftc.teamcode.RoperStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFMotorController {
    private double kp;
    private double ki;
    private double kd;
    private double kf;
    private double lastError = 0;
    private double derivative;
    private double error = 0;
    private ElapsedTime timer;
    DcMotor motor;
    private double midpoint;
    private double direction = 1;

    public PIDFMotorController(double kp, double ki, double kd, double kf, DcMotor motor, double midpoint ){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        timer = new ElapsedTime();
        this.motor = motor;
        this.midpoint = midpoint;
    }

    public double calculate(double target){
        double currentPos = this.motor.getCurrentPosition();
        error = target - currentPos;
        derivative = (error - lastError)/timer.seconds();
        lastError = error;
        return kp*error + kd*derivative + kf*direction*(midpoint-currentPos); //check direction for kf to combat gravity
    }
}
