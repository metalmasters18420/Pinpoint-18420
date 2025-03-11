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
import static org.firstinspires.ftc.teamcode.VariablesDelay.coords;
import static org.firstinspires.ftc.teamcode.VariablesDelay.off;
import static org.firstinspires.ftc.teamcode.VariablesDelay.pink;
import static org.firstinspires.ftc.teamcode.VariablesDelay.red;

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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Driver Control 2025 LIMEY", group = "1")
public class DriveControlLimelight extends  OpMode {


    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    public Limelight3A limelight;


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

    public enum camera{
        NONE,
        YELLOW,
        BLUE,
        RED
    }

    camera color = camera.NONE;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        limelight = hardwareMap.get(Limelight3A.class, "cam");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(2);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
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

        switch (color) {
            case NONE:

                if (gamepad1.y){
                    limelight.pipelineSwitch(0);
                    color = camera.YELLOW;
                }

                if (gamepad1.x){
                    color = camera.BLUE;
                }

                if (gamepad1.b){
                    color = camera.RED;
                }
            break;
            case YELLOW:

                limelight.pipelineSwitch(0);
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

                    hw.arm.setPosition(Arest);

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

//                    if (hw.lLift.getCurrentPosition() < 50){
                    bobot = robot.REST;
//                    }
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

                if (gamepad1.dpad_up){
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

                if (gamepad1.dpad_up) {

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

        //BEGIN LIMELIGHT

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Access classifier results
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }


        //END LIMELIGHT

//        telemetry.addData("power ", rPower);
//        telemetry.addData("rotation ", hw.rotation.ELC.getVoltage() * degree_per_volt);
//        telemetry.addData("armpos ", hw.arm.getPosition());
//        telemetry.addData("leftTarget",hw.lLift.getTargetPosition());
//        telemetry.addData("rightTarget",hw.rLift.getTargetPosition());
//        telemetry.addData("lift position", hw.lLift.getCurrentPosition());
//        telemetry.addData("bobot", bobot);
//        telemetry.addData("pos", hw.drive.pinpoint.getPositionRR());
//        telemetry.addData("Wrist ", hw.wrist.getPosition());




        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), hw.drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.update();

        }

        @Override
        public void stop(){

            coords = hw.drive.pinpoint.getPositionRR();

            hw.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
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
            hw.arm.setPosition(Adown);
            hw.wrist.setPosition(Wbin);
            hw.lift.LiftHang1();
            hw.rotation.RotateHang();
        }

        public void Hang2(){
            hw.rotation.RotateHang2();
        }
}