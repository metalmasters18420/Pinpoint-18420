package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Lift", group = "BrennanTestCode")

public class LiftExample extends  OpMode {

    private ElapsedTime timer = new ElapsedTime();
    private DcMotor ll;
    private DcMotor rl;

    public static double kP = 0.04;
    public static double kI = 0.001;
    public static double kD = 0;
    public static double kF = 0;
    double lastError = 0;
    double sum = 0;
    public static int threshold = 0;
    public static int summax = 100;

    public static int lowpos = 0;
    public static int highpos = 2000;
    public static int targetpos = lowpos;

public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rl = hardwareMap.get(DcMotor.class, "rl");
        rl.setDirection(DcMotorSimple.Direction.FORWARD);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rl.setPower(0);

        ll = hardwareMap.get(DcMotor.class, "ll");
        ll.setDirection(DcMotorSimple.Direction.REVERSE);
        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ll.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ll.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            targetpos = highpos;
        }
        if (gamepad1.dpad_down) {
            targetpos = lowpos;
        }
        int lcurrentpos = ll.getCurrentPosition();
        double out = lLiftControl(targetpos, lcurrentpos, threshold);

        telemetry.addData("loutput", out);
        telemetry.addData("targetpos", targetpos);
        telemetry.addData("Llift pos", lcurrentpos);
        telemetry.addData("lerror", targetpos - lcurrentpos);
        telemetry.addData("integralsum", sum);

        ll.setPower(out);
        rl.setPower(out);
    }

    @Override
    public void stop() {
    }

    double lLiftControl(int target, int current, int thresh) {

        int error = target - current;
        double deriv = (error - lastError) / timer.seconds();
        sum += error;

        if (sum > summax) {
            sum = summax;
        }
        if (sum < summax) {
            sum = -summax;
        }

        lastError = error;
        timer.reset();
        double output = kP * error + kD * deriv + kI * sum;

        if (Math.abs(error) < thresh) {
            return 0;
        } else {
            return output;
        }
    }
}
//    private final DcMotor leftLift;
//    private final DcMotor rightLift;
//
//    public int newTarget = 0;
//
//    public Lift(DcMotor ll, DcMotor rl){
//        this.leftLift = ll;
//        this.rightLift = rl;
//
//        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftLift.setTargetPosition(0);
//        rightLift.setTargetPosition(0);
//
//        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftLift.setPower(1);
//        rightLift.setPower(1);
//
//    }
//
//    public void Llowbin(){
//        newTarget = (int) (LIFTLBIN * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//    public void Lhighbin(){
//        newTarget = (int) (LIFTHBIN * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//    public void Lhighbar(){
//        newTarget = (int) (LIFTHBAR * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//    public void Llowbar(){
//        newTarget = (int) (LIFTLBAR * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//    public void Lwall(){
//        newTarget = (int) (LIFTWALL * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//    public void Lrest(){
//        newTarget = (int) (LIFTREST * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//    public void Lhang1(){
//        newTarget = (int) (LIFTHANG1 * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//    public void Lhang2(){
//        newTarget = (int) (LIFTHANG2 * CountsPerin);
//        leftLift.setTargetPosition(newTarget);
//        rightLift.setTargetPosition(newTarget);
//    }
//}