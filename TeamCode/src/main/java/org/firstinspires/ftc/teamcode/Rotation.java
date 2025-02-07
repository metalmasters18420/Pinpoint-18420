package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesRotate.Rrest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import dalvik.system.DelegateLastClassLoader;

@Config

public class Rotation {

    ElapsedTime timer = new ElapsedTime();
    public DcMotor Rotate;
    public AnalogInput Analog;
    public static double P = 0, I = 0, D = 0, F = 0;

    public final double degree_per_volt = 360 / 3.3;

    double lastError = 0;
    double sum = 0;
    public static int threshold = 0;
    public static int summax = 0;
    public static double target = Rrest;

    public Rotation(DcMotor r, AnalogInput a){
        this.Rotate = r;
        this.Analog = a;

        Rotate.setPower(0);
        Rotate.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    public void setTarget(double target) {
        Rotation.target = target;
    }

    public void Update(){
        double output = 0;
        double current = Analog.getVoltage();

        double error = target - current;
        double derivative = (error - lastError)/timer.seconds();

        lastError = error;
        timer.reset();


        double ff = Math.toRadians(current * degree_per_volt) * F;



        output = (P * error + D * derivative) + ff;

        Rotate.setPower(output);
    }
}
