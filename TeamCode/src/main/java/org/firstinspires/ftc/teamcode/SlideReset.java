package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class SlideReset extends robotBase{
    @Override
    protected void robotInit() {



    }

    @Override
    protected void robotInitLoop() {

    }

    @Override
    protected void robotStart(){

        slideSubsystem.setSlidePower(-gamepad2.right_stick_y);

    }
}
