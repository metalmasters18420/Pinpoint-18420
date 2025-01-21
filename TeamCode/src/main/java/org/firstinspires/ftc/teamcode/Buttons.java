package org.firstinspires.ftc.teamcode;

public class Buttons {}
//    hwRobot hw = new hwRobot();
//
//    Boolean x2Current = false;
//    Boolean x2Last = false;
//    Boolean x2Toggle = false;
//
//    Boolean y2Current = false;
//    Boolean y2Last = false;
//    Boolean y2Toggle = false;
//
//    Boolean a2Current = false;
//    Boolean a2Last = false;
//    Boolean a2Toggle = false;
//
//    public boolean x2button() {
//
//    x2Current = gamepad2.x;
//
//        if (x2Current && !x2Last){
//            x2Toggle = !x2Toggle;
//        }
//        if (x2Toggle){
//            hw.FlipClaw();
//        }
//        else{
//          hw.FlipIntake();
//        }
//
//        x2Last = x2Current;
//        return false;
//    }
//
//    public boolean y2button() {
//
//        y2Current = gamepad2.y;
//
//        if (y2Current && !y2Last) {
//            y2Toggle = !y2Toggle;
//        }
//        if (y2Toggle){
//            hw.Hextend();
//        }
//        else{
//            hw.HRetract();
//        }
//
//        y2Last = y2Current;
//        return false;
//    }
//
//    public boolean a2button() {
//
//        a2Current = gamepad2.a;
//
//        if (a2Current && !a2Last) {
//            a2Toggle = !a2Toggle;
//        }
//        if (a2Toggle){
//            hw.claw.setPosition(CLAW_CLOSED);
//        }
//        else{
//            hw.claw.setPosition(CLAW_OPEN);
//        }
//
//        a2Last = a2Current;
//        return false;
//    }
//}