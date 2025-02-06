package org.firstinspires.ftc.teamcode.RoperStuff.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoActionWithWait implements Action {
    Servo servo;
    double position;
    double actionDuration;
    ElapsedTime timer;

    public ServoActionWithWait(Servo s, double p, double w){
        this.servo = s;
        this.position = p;
        this.actionDuration = w;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (timer == null){
            timer = new ElapsedTime();
        }
        this.servo.setPosition(position);
        return timer.seconds() < actionDuration;
    }
}
