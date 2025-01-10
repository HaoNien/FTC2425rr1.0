package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;


import java.util.List;

@Config
public abstract class robotBase extends OpMode {
    protected MecanumDrive drive;
    protected ArmSubsystem armSubsystem;

    protected SlideSubsystem slideSubsystem;

    protected ClawSubsystem clawSubsystem;

    protected WristSubsystem wristSubsystem;








    public boolean isHangingMode = false; // 預設為普通模式
    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 1000)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 1000)  //  Pause for 300 mSec

            .build();
    double lift = 0, turn = 0;

    public boolean initDone=false;

    @Override
    public void init(){

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //drive = new MecanumDrive(hardwareMap,);
        armSubsystem = new ArmSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        clawSubsystem = new ClawSubsystem(hardwareMap);


        robotInit();
        initDone = true;
        telemetry.addData("init","done");
        telemetry.update();
    }
    public void init_loop(){
        if(initDone){
            robotInitLoop();
        }
    }
    public void loop(){
        robotStart();

    }


    protected abstract void robotInit();
    protected abstract void robotInitLoop();
    protected abstract void robotStart();






}
