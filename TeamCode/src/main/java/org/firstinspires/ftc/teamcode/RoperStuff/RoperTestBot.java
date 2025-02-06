package org.firstinspires.ftc.teamcode.RoperStuff;

import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.BUTTON_DELAY_TIME;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Roper Test Bot", group = "Roper")
public class RoperTestBot extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    ColorDetector colorDetector = null;

    ElapsedTime timer = new ElapsedTime();
    Boolean ButtonIsReady = true;

    @Override
    public void init() {
        this.colorDetector = new ColorDetector(hardwareMap);

    }

    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();



        packet.put("Color Detected Top",colorDetector.updateIndividualSensor(colorDetector.colorSensorTop));
        packet.put("Color Detected Bot",colorDetector.updateIndividualSensor(colorDetector.colorSensorBottom));
        packet.put("Color Detected Overall",colorDetector.detectedColor());

        float[] hsvT = colorDetector.readHSV(colorDetector.colorSensorTop);
        float[] hsvB = colorDetector.readHSV(colorDetector.colorSensorBottom);
        packet.put("Top H",hsvT[0]);
        packet.put("Top S",hsvT[1]);
        packet.put("Bot H",hsvB[0]);
        packet.put("Bot S",hsvB[1]);



        dash.sendTelemetryPacket(packet);



    }
}
