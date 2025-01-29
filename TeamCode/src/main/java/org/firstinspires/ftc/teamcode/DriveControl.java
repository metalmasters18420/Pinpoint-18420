package org.firstinspires.ftc.teamcode;

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
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rbar;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rbar2;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rbin;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rhang;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rhang2;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rin;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rrest;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rwall;

import android.view.animation.RotateAnimation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    public enum robot{
        REST,
        BIN,
        IN,
        HANG,
        HANG2
    }

    robot bobot = robot.REST;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
    }

    public void start() {
        clock.reset();
        light.reset();
        liftClock.reset();

        hw.init(hardwareMap);

    }

    @Override
    public void loop() {

        boolean Bdelay = clock.milliseconds() > ButtonDelay;
        boolean RGB = light.milliseconds() > .01;

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
                hw.spin.setPosition(hw.spin.getPosition() + .02);
            }
            if (gamepad1.left_bumper){
                hw.spin.setPosition(hw.spin.getPosition() - .02);
            }
            hw.Light.setPosition(LGREEN);
        }

        x1Last = x1Current;

        switch (bobot) {
            case REST:

                if (liftClock.milliseconds() > RotateDelay && liftClock.milliseconds() < 1000){
                    Rest();
                }

                if (gamepad2.dpad_up && Bdelay) {

                    Bin();
                    liftClock.reset();
                    bobot = robot.BIN;

                }
                if (gamepad2.dpad_right && Bdelay){

                    Hang();
                    liftClock.reset();
                    bobot = robot.HANG;

                }
                if (gamepad2.y && Bdelay){

                    Intake();
                    liftClock.reset();
                    bobot = robot.IN;

                }
                if (gamepad2.dpad_left && Bdelay){

                    Hang2();
                    liftClock.reset();
                    bobot = robot.HANG2;
                }
            break;
            case BIN:

                if (liftClock.milliseconds() > 1000 && liftClock.milliseconds() < 2000){
                    hw.lift.LiftBin();
                }

                if (gamepad2.dpad_down && Bdelay) {

                    hw.lift.LiftRest();
                    liftClock.reset();
                    bobot = robot.REST;

                }
                break;
            case HANG:

                if (liftClock.milliseconds() > RotateDelay && liftClock.milliseconds() < 2000){
                    hw.lift.LiftHang1();
                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
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
                    bobot = robot.REST;

                }
                break;
            case IN:

                if (liftClock.milliseconds() > RotateDelay && liftClock.milliseconds() < 1000){
                    hw.lift.LiftIn();
                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    bobot = robot.REST;

                }

                if (gamepad1.b && Bdelay){
                    hw.arm.setPosition(Adown);
                }
                else {
                    hw.arm.setPosition(Ain);
                }

                hw.lift.Move(gamepad2.left_stick_y);

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

        telemetry.addData("x", hw.drive.pose.position.x);
        telemetry.addData("y", hw.drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(hw.drive.pose.heading.toDouble()));

        telemetry.addData("leftTarget",hw.lLift.getTargetPosition());
        telemetry.addData("rightTarget",hw.rLift.getTargetPosition());
        telemetry.addData("leftTarget",hw.lLift.getCurrentPosition());
        telemetry.addData("rightTarget",hw.rLift.getCurrentPosition());
        telemetry.addData("power",hw.lLift.getPower());
        telemetry.addData("power",hw.rLift.getPower());
        telemetry.addData("a2Toggle",a2Toggle);

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), hw.drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("lift position", hw.lLift.getCurrentPosition());
        telemetry.addData("rotation", hw.rrotate.getPosition());
        telemetry.addData("bobot", bobot);

        telemetry.update();
        }

        public void Rest(){
            hw.arm.setPosition(Arest);
            hw.wrist.setPosition(Wrest);
            hw.lrotate.setPosition(Rrest);
            hw.rrotate.setPosition(Rrest);
        }

        public void Wall(){
            hw.arm.setPosition(Awall);
            hw.wrist.setPosition(Wwall);
            hw.lrotate.setPosition(Rwall);
            hw.rrotate.setPosition(Rwall);
        }

        public void Bin(){
            hw.arm.setPosition(Abin);
            hw.wrist.setPosition(Wbin);
            hw.lrotate.setPosition(Rbin);
            hw.rrotate.setPosition(Rbin);
        }

        public void Bar(){
            hw.arm.setPosition(Abar);
            hw.wrist.setPosition(Wbar);
            hw.lrotate.setPosition(Rbar);
            hw.rrotate.setPosition(Rbar);
        }

        public void Bar2(){
        hw.arm.setPosition(Abar2);
        hw.wrist.setPosition(Wbar2);
        hw.lrotate.setPosition(Rbar2);
        hw.rrotate.setPosition(Rbar2);
        }

        public void Intake(){
            hw.arm.setPosition(Ain);
            hw.wrist.setPosition(Win);
            hw.lrotate.setPosition(Rin);
            hw.rrotate.setPosition(Rin);
        }

        public void Hang() {
            hw.arm.setPosition(Arest);
            hw.lift.LiftRest();
            hw.lrotate.setPosition(Rhang);
            hw.rrotate.setPosition(Rhang);
        }

        public void Hang2(){
        hw.rrotate.setPosition(Rhang2);
        hw.lrotate.setPosition(Rhang2);
        }
}