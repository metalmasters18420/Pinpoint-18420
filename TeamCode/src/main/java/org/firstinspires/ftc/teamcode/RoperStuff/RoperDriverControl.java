package org.firstinspires.ftc.teamcode.RoperStuff;

import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.BUTTON_DELAY_TIME;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class RoperDriverControl extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private final List<Action> runningActions = new ArrayList<>();

    Robot r = null;

    ElapsedTime timer = new ElapsedTime();
    Boolean ButtonIsReady = true;

    @Override
    public void init() {
        this.r = new Robot(hardwareMap);

    }

    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
        ButtonIsReady = timer.seconds()> BUTTON_DELAY_TIME;
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        /*
        if (gamepad1.b && ButtonIsReady){
            runningActions.add(r.openClaw());
        }


        // update running actions - boiler plate code.
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
         */

        runningActions.removeIf((a) -> {
            a.preview(packet.fieldOverlay());
            return !a.run(packet);
        });

        packet.put("Color Detected Top",r.colorDetector.updateIndividualSensor(r.colorDetector.colorSensorTop));
        packet.put("Color Detected Bot",r.colorDetector.updateIndividualSensor(r.colorDetector.colorSensorBottom));
        packet.put("Color Detected Top",r.colorDetector.detectedColor());

        float[] hsvT = r.colorDetector.readHSV(r.colorDetector.colorSensorTop);
        float[] hsvB = r.colorDetector.readHSV(r.colorDetector.colorSensorBottom);
        packet.put("Top H",hsvT[0]);
        packet.put("Top S",hsvT[1]);
        packet.put("Bot H",hsvB[0]);
        packet.put("Bot S",hsvB[1]);

        dash.sendTelemetryPacket(packet);



    }
}
