package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous
public class AUTOTest extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));

    @Override
    protected void robotInit() {
        Pose2d startPose = new Pose2d(-10, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        //armTarget= 45;
        slideTarget = 40;
        lift=-90;
        turn = 0;
        Claw.setPosition(claw_Close);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(() -> {
                    lift=50;
                    armTarget= 37;
                    slideTarget = 75;
                })

                .forward(27)
                .addTemporalMarker(() -> {        Claw.setPosition(claw_Open);
                })
                .back(20)
                .addTemporalMarker(() -> {
                    armTarget=5;
                    lift=-90;
                    turn=45;
                })
                .turn(Math.toRadians(-45))
                .splineTo(new Vector2d(-29,41),Math.toRadians(220))
                .addTemporalMarker(() -> {
                    armTarget=0;
                    lift=-90;
                    turn=45;
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {Claw.setPosition(claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armTarget=5;
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    Claw.setPosition(claw_Open);


                })
                .waitSeconds(0.5)

                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-39,41))
                .addTemporalMarker(() -> {armTarget=0;})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Claw.setPosition(claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armTarget=5;
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    Claw.setPosition(claw_Open);



                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(90))


                .strafeTo(new Vector2d(-49,41))
                .addTemporalMarker(() -> {armTarget=0;})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Claw.setPosition(claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {

                    armTarget=5;
                    slideTarget= 40;
                })
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-49,51))
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    armTarget=20;
                    lift = -10;
                    turn=0;
                    Claw.setPosition(claw_Open);
                })
                .waitSeconds(0.5)





                //.strafeLeft(20)
                //.strafeTo(new Vector2d(-38, 12))
                //.strafeTo(new Vector2d(-45, 50))
                //.strafeTo(new Vector2d(-45, 12))


                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        telemetry.addData("Arm Po000000000000000000000000000000000000000000000","");
        telemetry.update();



    }

    @Override
    protected void robotInitLoop() {
        armSubsystem.moveToAngle();                       // 將手臂維持在目標角度


    }

    @Override
    protected void robotStart(){

        drive.update();
        armTurn2angle(armTarget);
        slideSubsystem.slideToPosition();
        slideToPosition(slideTarget);
        wristToPosition(lift, turn);



    }
}
