package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

@Config
@TeleOp(name = "PotatoLinOpMode", group = "Linear OpMode")
public class PotatoLinOpMode extends robotBase {

    private final ElapsedTime runtime = new ElapsedTime();
    public static double slide_Speed = 10;
    public static double arm_Speed = 5;
    private long lastTime = System.currentTimeMillis();
    private int loopCount = 0, loopCountHZ = 0;

    // Gamepad & ButtonReader
    private GamepadEx gamepadEx2;
    private ButtonReader bButtonReader, yButtonReader, xButtonReader, aButtonReader, leftBumperReader, rightBumperReader;

    // 狀態機枚舉類型與當前狀態
    private enum RobotState {STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7, STATE_8}

    private RobotState currentStateB = RobotState.STATE_7;
    private RobotState currentStateY = RobotState.STATE_7;
    private RobotState currentStateX = RobotState.STATE_7;
    private RobotState currentStateA = RobotState.STATE_1;

    @Override
    public void robotInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        wristSubsystem.wristToPosition(-90, 0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // 初始化 GamepadEx 和按鈕監聽器
        gamepadEx2 = new GamepadEx(gamepad2);
        bButtonReader = new ButtonReader(gamepadEx2, GamepadKeys.Button.B);
        yButtonReader = new ButtonReader(gamepadEx2, GamepadKeys.Button.Y);
        xButtonReader = new ButtonReader(gamepadEx2, GamepadKeys.Button.X);
        aButtonReader = new ButtonReader(gamepadEx2, GamepadKeys.Button.A);
        leftBumperReader = new ButtonReader(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER);
        rightBumperReader = new ButtonReader(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER);
    }

    @Override
    protected void robotInitLoop() {
        armSubsystem.setArmTarget(45);
        armSubsystem.moveToAngle();
        telemetry.addData("Arm Position", armSubsystem.getArmPosNow());
        telemetry.addData("Target Angle", armSubsystem.getArmTarget());
        telemetry.update();
    }

    @Override
    public void robotStart() {
        loopCount++;
        // 讀取按鈕狀態
        bButtonReader.readValue();
        yButtonReader.readValue();
        xButtonReader.readValue();
        aButtonReader.readValue();
        leftBumperReader.readValue();
        rightBumperReader.readValue();

        // Claw 控制
        if (leftBumperReader.wasJustPressed()) clawSubsystem.openClaw();
        if (rightBumperReader.wasJustPressed()) clawSubsystem.closeClaw();

        // 管理按鈕狀態機
        if (bButtonReader.wasJustPressed()) switchStateB();
        if (yButtonReader.wasJustPressed()) switchStateY();
        if (xButtonReader.wasJustPressed()) switchStateX();
        if (aButtonReader.wasJustPressed()) switchStateA();

        // 狀態機執行
        executeStateLogic();

        // 控制 Slide 與 Arm
        controlSlide();
        controlArm();
        controlWrist();
        driveRobot();
        loopcount();
        // 顯示數據
        showTelemetryData();
    }

    // 控制 Slide
    private void controlSlide() {
        double gp2_r_Y = gamepad2.right_stick_y;
        if (Math.abs(gp2_r_Y) > 0.1) {
            slideSubsystem.setTarget(slideSubsystem.getSlidePosNow() + (-gp2_r_Y * slide_Speed)) ;
        }
        slideSubsystem.moveToPosition(armSubsystem.getArmPosNow());
    }

    // 控制 Arm
    private void controlArm() {
        armSubsystem.setSlidePosNow(slideSubsystem.getSlidePosNow());
        double gp2_l_Y = gamepad2.left_stick_y;
        if (Math.abs(gp2_l_Y) > 0.3) {
            armSubsystem.setArmTarget(armSubsystem.getArmPosNow() + (gp2_l_Y * arm_Speed));
        }

        armSubsystem.moveToAngle();
    }

    // 控制手腕
    private void controlWrist() {
        if (gamepad2.dpad_up) lift += 5;
        else if (gamepad2.dpad_down) lift -= 5;

        if (gamepad2.dpad_left) turn += 5;
        else if (gamepad2.dpad_right) turn -= 5;

        lift = clamp(lift, wristSubsystem.getLiftMin(), wristSubsystem.getLiftMax());
        turn = clamp(turn, wristSubsystem.getTurnMin(), wristSubsystem.getTurnMax());
        wristSubsystem.wristToPosition(lift, turn);
    }

    // 駕駛機器人
    private void driveRobot() {
        double axial = adjustSpeed(gamepad1.left_stick_y);
        double lateral = adjustSpeed(gamepad1.left_stick_x);
        double yaw = adjustYaw(gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.right_trigger);
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(axial,lateral),yaw));
    }

    private double adjustSpeed(double input) {
        return (Math.abs(input) < 0.7 && Math.abs(input) > 0.1) ? input / 2 : input;
    }

    private double adjustYaw(double input, double leftTrigger, double rightTrigger) {
        return input * 1.33 + (leftTrigger / 3) - (rightTrigger / 3);
    }
    private void loopcount(){
        long currentTime = System.currentTimeMillis();

        if (currentTime - lastTime >= 1000) {
            loopCountHZ = loopCount;


            loopCount = 0; // 重置計數器
            lastTime = currentTime; // 重置時間戳
        }
    }
    // 狀態機邏輯執行
    private void executeStateLogic() {
        executeStateLogicB();
        executeStateLogicY();
        executeStateLogicX();
        executeStateLogicA();
    }

    // 切換與執行狀態邏輯
    private void switchStateB() {
        currentStateB = currentStateB == RobotState.STATE_7 ? RobotState.STATE_1 : RobotState.values()[currentStateB.ordinal() + 1];
        currentStateY = RobotState.STATE_7; // 重置另一個模式的狀態
        currentStateX = RobotState.STATE_8; // 重置另一個模式的狀態

    }

    private void executeStateLogicB() {
        switch (currentStateB) {
            case STATE_1:
                setArmSlidePosition(20, 40, -20, 0, true);
                break;
            case STATE_2:
                clawSubsystem.closeClaw();
                break;
            case STATE_3:
                armSubsystem.setArmTarget(45);
                break;
            case STATE_4:
                setArmSlidePosition(90, 52, 90, 0, false);
                break;
            case STATE_5:
                slideSubsystem.setTarget(68);
                break;
            case STATE_6:
                clawSubsystem.openClaw();
                break;

            case STATE_7:
                clawSubsystem.openClaw();
                break;

        }
    }

    private void switchStateY() {
        currentStateY = currentStateY == RobotState.STATE_7 ? RobotState.STATE_1 : RobotState.values()[currentStateY.ordinal() + 1];
        currentStateB = RobotState.STATE_7; // 重置另一個模式的狀態
        currentStateX = RobotState.STATE_8; // 重置另一個模式的狀態
    }

    private void executeStateLogicY() {
        switch (currentStateY) {
            case STATE_1:
                setArmSlidePosition(10, 70, 0, 0, true);
                break;
            case STATE_2:
                setArmSlidePosition(10, slideSubsystem.getSmax(), -90, 0, false);
                break;
            case STATE_3:
                clawSubsystem.closeClaw();
                break;
            case STATE_4:
                setArmSlidePosition(20, 40, 0, 0, true);
                break;
            case STATE_5:
                setArmSlidePosition(90, slideSubsystem.getSmax(), 20, 0, false);
                break;
            case STATE_6:
                clawSubsystem.openClaw();
                break;
                case STATE_7:
                setArmSlidePosition(25, 40, 0, 0, true);
                break;
        }
    }

    private void switchStateX() {
        currentStateX = currentStateX == RobotState.STATE_8 ? RobotState.STATE_1 : RobotState.values()[currentStateX.ordinal() + 1];
        currentStateY = RobotState.STATE_1; // 重置另一個模式的狀態
        currentStateB = RobotState.STATE_1; // 重置另一個模式的狀態
    }

    private void executeStateLogicX() {
        switch (currentStateX) {
            case STATE_1:
                setArmSlidePosition(10, 70, -90, 0, true);
                break;
            case STATE_2:
                clawSubsystem.closeClaw();
                break;
            case STATE_3:
                setArmSlidePosition(10, 40, -10, 0, false);
                break;
            case STATE_4:
                clawSubsystem.openClaw();
                break;
            case STATE_5:
                clawSubsystem.closeClaw();
                break;
            case STATE_6:
                setArmSlidePosition(50, 65, -10, 0, false);
                break;
            case STATE_7:
                armSubsystem.setArmTarget(35);
                break;
            case STATE_8:
                clawSubsystem.openClaw();
                break;

        }
    }

    private void switchStateA() {
        currentStateA = currentStateA == RobotState.STATE_2 ? RobotState.STATE_1 : RobotState.STATE_2;
        currentStateY = RobotState.STATE_1; // 重置另一個模式的狀態
        currentStateB = RobotState.STATE_1; // 重置另一個模式的狀態

    }

    private void executeStateLogicA() {
        switch (currentStateA) {
            case STATE_1:
                setArmSlidePosition(10, 70, -90, 0, true);
                break;
            case STATE_2:
                setArmSlidePosition(10, 70, -90, 0, true);
                break;
        }
    }
    private void setArmSlidePosition(double armTarget, double slideTarget, double lift, double turn, boolean isClawOpen) {
        armSubsystem.setArmTarget(armTarget);
        slideSubsystem.setTarget(slideTarget);
        wristSubsystem.wristToPosition(lift, turn);
        if (isClawOpen) clawSubsystem.openClaw();
        else clawSubsystem.closeClaw();
    }

    private void showTelemetryData() {
        double[] currents = armSubsystem.getCurrent();
        telemetry.addData("Loop Frequency", "%d Hz", loopCountHZ);
        telemetry.addData("Slide Position", slideSubsystem.getSlidePosNow());
        telemetry.addData("Arm Current L/R", "%f / %f", currents[0], currents[1]);
    }
}
