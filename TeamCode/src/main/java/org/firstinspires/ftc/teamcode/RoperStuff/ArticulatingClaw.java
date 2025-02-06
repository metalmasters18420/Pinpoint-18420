package org.firstinspires.ftc.teamcode.RoperStuff;

import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.CLAW_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.WRIST_DEPOSIT_POSITION;
import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.WRIST_GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.WRIST_LEVEL_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoperStuff.Actions.ServoActionWithWait;

public class ArticulatingClaw {

    enum State {READY,BUSY}
    State state;
    private Servo claw = null;
    private Servo wrist = null;

    public ArticulatingClaw(Servo claw, Servo wrist){
        this.claw = claw;
        this.wrist = wrist;
        state = State.READY;
    }

    Action grab(){return new ServoActionWithWait(claw,CLAW_CLOSED_POSITION,.3);}
    Action open(){return new ServoActionWithWait(claw,CLAW_OPEN_POSITION,.2);}
    Action moveWristToGrab(){return new ServoActionWithWait(wrist,WRIST_GRAB_POSITION,.3);}
    Action moveWristToLevel(){return new ServoActionWithWait(wrist,WRIST_LEVEL_POSITION,.1);}
    Action moveWristToDeposit(){return new ServoActionWithWait(wrist,WRIST_DEPOSIT_POSITION,.3);}

}
