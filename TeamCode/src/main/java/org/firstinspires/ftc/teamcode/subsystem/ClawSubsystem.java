package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawSubsystem extends SubsystemBase {
    private Servo claw;


    public static double claw_Open = 0.5;

    public static double claw_Close = 0.22;

    public ClawSubsystem(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");


    }
    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(claw_Close);
            return false;
        }
    }
    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(claw_Open);
            return false;
        }
    }
    public Action openClaw() {
        return new OpenClaw();
    }
}
