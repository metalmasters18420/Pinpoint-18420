package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Rotation.degree_per_volt;
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
import static org.firstinspires.ftc.teamcode.VariablesDelay.pink;
import static org.firstinspires.ftc.teamcode.VariablesDelay.off;
import static org.firstinspires.ftc.teamcode.VariablesDelay.red;
import static org.firstinspires.ftc.teamcode.VariablesDelay.coords;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rhang2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Driver Control 2025", group = "1")
public class DriveControl extends  OpMode {


    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();


    public ElapsedTime clock = new ElapsedTime();
    public ElapsedTime light = new ElapsedTime();
    public ElapsedTime liftClock = new ElapsedTime();

    hwRobot hw = new hwRobot();

    boolean a2Current = false, a2Last = false, a2Toggle = false;

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
        HANG2,
        RESET
    }

    robot bobot = robot.REST;

    public enum vroom{
        AUTO,
        MANUAL
    }

    vroom mode = vroom.MANUAL;


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
        hw.drive.pinpoint.setPositionRR(coords);

    }

    @Override
    public void loop() {

        boolean Bdelay = clock.milliseconds() > ButtonDelay;
        boolean RGB = light.milliseconds() > .01;

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
            }
            else {
                if (gamepad1.right_bumper){
                    hw.spin.setPosition(hw.spin.getPosition() + .08);
                }
                if (gamepad1.left_bumper){
                    hw.spin.setPosition(hw.spin.getPosition() - .08);
                }
            }

        x1Last = x1Current;

        if (mode == vroom.AUTO){
            hw.Light.setPosition(pink);
        }
        else if (mode == vroom.MANUAL && x1Toggle){
            hw.Light.setPosition(red);
        }
        else {
            hw.Light.setPosition(off);
        }

        if (gamepad2.x && gamepad2.b){
            hw.lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hw.rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        switch (bobot) {
            case REST:

                if (liftClock.milliseconds() > 300 && liftClock.milliseconds() < 2000){
                    Rest();
                }

                if (gamepad2.dpad_up && Bdelay) {

                    hw.rotation.RotateRest();
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
                if (gamepad2.y && Bdelay){

                    liftClock.reset();
                    clock.reset();
                    bobot = robot.RESET;
                }
            break;
            case RESET:

                hw.lift.Move(gamepad2.right_trigger - gamepad2.left_trigger);

                if (gamepad2.x && gamepad2.b){
                    hw.lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hw.rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    hw.rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hw.lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad2.dpad_down && Bdelay) {

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    Rest();
                    bobot = robot.REST;

                }

            break;
            case BIN:

                if (hw.lLift.getCurrentPosition() >= 1300){
                    Bin();
                }

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

                if (gamepad2.y && Bdelay){

                    liftClock.reset();
                    clock.reset();
                    bobot = robot.RESET;
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
                if (gamepad2.y && Bdelay){

                    liftClock.reset();
                    clock.reset();
                    bobot = robot.RESET;
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
                if (gamepad2.y && Bdelay){

                    liftClock.reset();
                    clock.reset();
                    bobot = robot.RESET;
                }
                break;
//            case BAR2:
            case HANG:

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.REST;

                }

                if (gamepad2.dpad_left && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.HANG2;

                }
                if (gamepad2.y && Bdelay){

                    liftClock.reset();
                    clock.reset();
                    bobot = robot.RESET;
                }
                break;
            case HANG2:

                if (liftClock.milliseconds() > 500){
                    Hang2();
                }

                if (gamepad2.dpad_down && Bdelay){

                    hw.lift.LiftRest();
                    liftClock.reset();
                    clock.reset();
                    bobot = robot.REST;

                }
                break;
            case IN:

                if (gamepad2.dpad_down){

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

                if (gamepad2.y && Bdelay){

                    liftClock.reset();
                    clock.reset();
                    bobot = robot.RESET;
                }
            break;
            default:
                bobot = robot.REST;
        }

        hw.rotation.loop();


        TelemetryPacket packet = new TelemetryPacket();

        switch (mode) {
            case MANUAL:
                hw.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                if (gamepad1.y){
                    mode = vroom.AUTO;
                }
                break;
            case AUTO:

                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
            }
                runningActions = newActions;

                dash.sendTelemetryPacket(packet);

                if (gamepad1.y) {

                    TrajectoryActionBuilder drivetobucket = hw.drive.actionBuilder(hw.drive.pinpoint.getPositionRR())
                            .strafeToLinearHeading(new Vector2d(-52, -49), Math.toRadians(45), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-40, 70))
                            .endTrajectory();

                    Action ToBucket = drivetobucket.build();

                    runningActions.add(new SequentialAction(
                            ToBucket,
                            new InstantAction(() -> mode = vroom.MANUAL)
                    ));

                }

                if (gamepad1.dpad_left){
                    mode = vroom.MANUAL;
                }

                break;
                default:
                    mode = vroom.MANUAL;
        }

        hw.drive.updatePoseEstimate();


        telemetry.addData("power ", rPower);
        telemetry.addData("rotation ", hw.rotation.ELC.getVoltage() * degree_per_volt);
        telemetry.addData("armpos ", hw.arm.getPosition());
        telemetry.addData("leftTarget",hw.lLift.getTargetPosition());
        telemetry.addData("rightTarget",hw.rLift.getTargetPosition());
        telemetry.addData("lift position", hw.lLift.getCurrentPosition());
        telemetry.addData("bobot", bobot);
        telemetry.addData("pos", hw.drive.pinpoint.getPositionRR());

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), hw.drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.update();

        }

        @Override
        public void stop(){
            coords = hw.drive.pinpoint.getPositionRR();
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
            hw.rotation.RotateHang2();
        }
}