package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesRotate.Rin;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rrest;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
    public class Rotation extends SubsystemBase {
        private PIDController controller;

        public static double p = 0.01, i = 0, d = 0.001, f = 0.003;

        public static double target = 0;
        public final double degree_per_volt = 360 / 3.3;

        public AnalogInput ELC;
        public DcMotor Rotate;

        public Rotation(DcMotorEx r, AnalogInput a){

            controller = new PIDController(p, i, d);
            controller.setPID(p, i, d);

            Rotate = r;
            ELC = a;

//            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//            Rotate = hardwareMap.get(DcMotorEx.class, "R");
//            ELC = hardwareMap.get(AnalogInput.class, "ELC");

            Rotate.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public void loop() {

            double armPos = ELC.getVoltage() * degree_per_volt;
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target * degree_per_volt)) * f;
            double power = pid + ff;

            Rotate.setPower(power);
//            telemetry.addData("pos ", armPos);
//            telemetry.addData("target ", target);
//            telemetry.addData("power ", pid);
//            telemetry.update();

        }

        public void RotateRest() {
            target = Rrest;
        }

        public void RotateIn() {
            target = Rin;
        }


//    ElapsedTime timer = new ElapsedTime();
//    public DcMotor Rotate;
//    public AnalogInput Analog;
//    public static double P = 0.1, I = 0, D = 0, F = 0;
//
//    public final double degree_per_volt = 360 / 3.3;
//
//    double lastError = 0;
//    double sum = 0;
//    public static int threshold = 0;
//    public static int summax = 0;
//    public static double target = Rrest;
//
//    public Rotation(DcMotor r, AnalogInput a){
//        this.Rotate = r;
//        this.Analog = a;
//
//        Rotate.setPower(0);
//        Rotate.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        timer.reset();
//    }
//
//    public void setTarget(double target) {
//        Rotation.target = target;
//    }
//
//    public void Update(){
//
//        double current = Analog.getVoltage();
//
//        double error = target - current;
//        double derivative = (error - lastError)/timer.seconds();
//
//        lastError = error;
//        timer.reset();
//
//
//        double ff = Math.toRadians(current * degree_per_volt) * F;
//
//
//
//        double output = (P * error + D * derivative) + ff;
//
//        Rotate.setPower(output);
//    }
    }
