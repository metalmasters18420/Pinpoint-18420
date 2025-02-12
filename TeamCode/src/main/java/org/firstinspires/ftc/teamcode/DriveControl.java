package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Rotation.rPower;
import static org.firstinspires.ftc.teamcode.VariablesArm.Abar;
import static org.firstinspires.ftc.teamcode.VariablesArm.Abar2;
import static org.firstinspires.ftc.teamcode.VariablesArm.Abin;
import static org.firstinspires.ftc.teamcode.VariablesArm.Adown;
import static org.firstinspires.ftc.teamcode.VariablesArm.Ain;
import static org.firstinspires.ftc.teamcode.VariablesArm.Arest;
import static org.firstinspires.ftc.teamcode.VariablesArm.Awall;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Cclose;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Copen;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Srest;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wbar;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wbar2;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wbin;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Win;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wrest;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wwall;
import static org.firstinspires.ftc.teamcode.VariablesDelay.ButtonDelay;
import static org.firstinspires.ftc.teamcode.VariablesDelay.LGREEN;
import static org.firstinspires.ftc.teamcode.VariablesDelay.LRED;
import static org.firstinspires.ftc.teamcode.VariablesDelay.RotateDelay;

import com.arcrobotics.ftclib.controller.PIDController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;

@Config
@TeleOp(name = "Driver Control 2025", group = "1")
public class DriveControl extends  OpMode {

    public ElapsedTime clock = new ElapsedTime();
    public ElapsedTime light = new ElapsedTime();
    public ElapsedTime liftClock = new ElapsedTime();

    hwRobot hw = new hwRobot();

    boolean a2Current = false;
    boolean a2Last = false;
    boolean a2Toggle = false;

    boolean x1Current = false;
    boolean x1Last = false;
    boolean x1Toggle = false;

    boolean a1Current = false, a1Last = false, a1Toggle = false;

    public enum robot{
        REST,
        BIN,
        BAR,
        BAR2,
        WALL,
        IN,
        HANG,
        HANG2
    }

    robot bobot = robot.REST;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");

//        hw.controller = new PIDController(p, i, d);
//        hw.controller.setPID(p, i, d);
    }

    public void start() {
        clock.reset();
        light.reset();
        liftClock.reset();

        hw.init(hardwareMap);

    }

    @Override
    public void loop() {

//        double armPos = hw.armEncoder.getVoltage() * degree_per_volt;
//        double pid = hw.controller.calculate(armPos, target);
//        double power = pid; //+ ff;
////        rPower = power;
//
//        hw.Rotate.setPower(power);

        boolean Bdelay = clock.milliseconds() > ButtonDelay;
        boolean RGB = light.milliseconds() > .01;

        a2Current = gamepad2.a;
        double encoderVal = hw.armEncoder.getVoltage();

//        a1Current = gamepad1.a;
//
//        if (a1Current && ! a1Last){
//            a1Toggle = !a1Toggle;
//        }
//        if (a1Toggle){
//            hw.rotation.RotateIn();
//        }
//        else {
//            hw.rotation.RotateRest();
//        }
//
//        a1Last = a1Current;

        a2Current = gamepad2.a;

        if (a2Current && !a2Last){
            a2Toggle = !a2Toggle;
        }
        if (a2Toggle){
            hw.claw.setPosition(Cclose);
        }
        else {
            hw.claw.setPosition(Copen);
        }

        a2Last = a2Current;

        x1Current = gamepad1.x;

        if (x1Current && !x1Last){
            x1Toggle = !x1Toggle;
        }
        if (x1Toggle){
            hw.spin.setPosition(Srest);
            hw.Light.setPosition(LRED);
        }
        else {
            if (gamepad1.right_bumper){
                hw.spin.setPosition(hw.spin.getPosition() + .08);
            }
            if (gamepad1.left_bumper){
                hw.spin.setPosition(hw.spin.getPosition() - .08);
            }
            hw.Light.setPosition(LGREEN);
        }

        x1Last = x1Current;

        if (gamepad2.x && gamepad2.y){
            hw.lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hw.rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        switch (bobot) {
            case REST:

                if (liftClock.milliseconds() > 100 && liftClock.milliseconds() < 2000){
                    Rest();
                }

                if (gamepad2.dpad_up && Bdelay) {

                    Bin();
                    hw.lift.LiftBin();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.BIN;

                }
                if (gamepad2.dpad_left && Bdelay){

                    Hang();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.HANG;

                }
                if (gamepad2.dpad_right && Bdelay){

                    Intake();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.IN;

                }
                if (gamepad2.left_bumper && Bdelay){

                    Wall();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.WALL;

                }
            break;
            case BIN:
//                if (liftClock.milliseconds() > 500 && liftClock.milliseconds() < 2000){
//                    hw.lift.LiftBin();
//                }

                if (gamepad2.dpad_down && Bdelay) {

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    Rest();
                    bobot = robot.REST;

                }

                if (gamepad2.dpad_left && Bdelay){

                    Hang();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.HANG;

                }
                break;
            case WALL:

                if (liftClock.milliseconds() > 500 && liftClock.milliseconds() < 2000){
                    hw.lift.LiftRest();
                }

                if (gamepad2.left_bumper && Bdelay){

                    hw.arm.setPosition(Abar);
                    hw.wrist.setPosition(Wbar);
                    hw.lift.LiftBar();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.BAR;
                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.REST;

                }
                if (gamepad2.dpad_right && Bdelay){

                    Intake();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.IN;

                }

                break;
            case BAR:

                if (gamepad2.left_bumper && Bdelay){

                    hw.arm.setPosition(Awall);
                    hw.wrist.setPosition(Wwall);
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.WALL;
                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.REST;

                }
                if (gamepad2.dpad_right && Bdelay){

                    Intake();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.IN;

                }
                break;
//            case BAR2:
//
//                if (gamepad2.y && Bdelay){
//
//                    Rest();
//                    liftClock.reset();
//                    bobot = robot.REST;
//
//                }
            case HANG:

//                if (liftClock.milliseconds() > RotateDelay && liftClock.milliseconds() < 2000){
//                    hw.lift.LiftHang1();
//                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.REST;

                }
                break;
            case HANG2:

                if (liftClock.milliseconds() > RotateDelay && liftClock.milliseconds() < 3000){
                    hw.lift.LiftHang2();
                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    Rest();
                    bobot = robot.REST;

                }
                break;
            case IN:

//                if (liftClock.milliseconds() > RotateDelay && liftClock.milliseconds() < 1000){
//                    hw.lift.LiftIn();
//                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();

                    if (hw.lLift.getCurrentPosition() < 50){
                        bobot = robot.REST;
                    }
                }
                else {
                    hw.lift.Move(gamepad2.right_trigger - gamepad2.left_trigger);
                }

                if (gamepad1.b && Bdelay){
                    hw.arm.setPosition(Adown);
                }
                else {
                    hw.arm.setPosition(Ain);
                }
            break;
            default:
                bobot = robot.REST;
        }

        hw.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
                ));

        hw.drive.updatePoseEstimate();


        hw.rotation.loop();

         telemetry.addData("power ", rPower);

//        telemetry.addData("x", hw.drive.pose.position.x);
//        telemetry.addData("y", hw.drive.pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(hw.drive.pose.heading.toDouble()));
//        telemetry.addData("Encoder Voltage", encoderVal);
        telemetry.addData("armpos ", hw.arm.getPosition());
        telemetry.addData("leftTarget",hw.lLift.getTargetPosition());
        telemetry.addData("rightTarget",hw.rLift.getTargetPosition());
//        telemetry.addData("power",hw.lLift.getPower());
//        telemetry.addData("a2Toggle",a2Toggle);


        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), hw.drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("lift position", hw.lLift.getCurrentPosition());
//        telemetry.addData("rotation", hw.rrotate.getPosition());
        telemetry.addData("bobot", bobot);

        telemetry.update();

        }

        public void Rest(){
            hw.arm.setPosition(Arest);
            hw.wrist.setPosition(Wrest);
            hw.rotation.RotateRest();
        }

        public void Wall(){
            hw.arm.setPosition(Awall);
            hw.wrist.setPosition(Wwall);
            hw.rotation.RotateWall();
        }

        public void Bin(){
            hw.arm.setPosition(Abin);
            hw.wrist.setPosition(Wbin);
            hw.rotation.RotateRest();
        }

        public void Bar(){
            hw.arm.setPosition(Abar);
            hw.wrist.setPosition(Wbar);
            hw.rotation.RotateBar();
        }

        public void Bar2(){
        hw.arm.setPosition(Abar2);
        hw.wrist.setPosition(Wbar2);
//        hw.lrotate.setPosition(Rbar2);
//        hw.rrotate.setPosition(Rbar2);
        }

        public void Intake(){
            hw.arm.setPosition(Ain);
            hw.wrist.setPosition(Win);
            hw.rotation.RotateIn();
        }

        public void Hang() {
            hw.arm.setPosition(Arest);
            hw.lift.LiftHang1();
            hw.rotation.RotateHang();
        }

        public void Hang2(){
//        hw.rrotate.setPosition(Rhang2);
//        hw.lrotate.setPosition(Rhang2);
        }
}