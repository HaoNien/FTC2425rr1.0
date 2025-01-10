package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * ArmSubsystem 負責控制機器手臂的子系統，實現手臂角度的 PID 控制和電機輸出。
 */
@Config
public class ArmSubsystem {
    private DcMotorEx armL;  // 左手臂電機
    private DcMotorEx armR;  // 右手臂電機
    private Encoder armEnc;  // 編碼器
    private PIDController ArmPID;  // PID 控制器

    // 參數
    private double armTarget = 45;  // 手臂的目標角度
    private double armP = 0.08;     // PID 比例參數
    private double armI = 0.1;      // PID 積分參數
    private double armD = 0.004;    // PID 微分參數
    private double armF = 0;        // 前饋參數
    private double arm_f_coeff = 0.0053;  // 重力補償係數
    private double armOutput;  // 電機輸出功率
    private double armPosNow;  // 當前手臂位置角度

    private double slidePosNow;  // 當前滑軌位置角度
    private static final double ARM_UP_LIMIT = 95;  // 手臂上限角度
    private static final double ARM_BOTTOM_LIMIT = 0;  // 手臂下限角度
    private static final double ARM_POWER_MAX = 1;  // 最大功率
    private static final double ARM_POWER_MIN = -0.4;  // 最小功率
    private static final double ARM_ENC_TO_DEG = 8192 / 360;  // 編碼器刻度到角度轉換係數
    private boolean hangMode = false;  // 是否處於掛鉤模式

    /**
     * ArmSubsystem 建構子。
     * 初始化電機、編碼器和 PID 控制器。
     *
     * @param hardwareMap 硬體映射表，用來取得機器的硬體元件
     */
    public ArmSubsystem(HardwareMap hardwareMap) {
        armL = hardwareMap.get(DcMotorEx.class, "armL");
        armR = hardwareMap.get(DcMotorEx.class, "armR");
        armL.setDirection(DcMotorSimple.Direction.REVERSE);  // 左臂電機方向反轉
        armR.setDirection(DcMotorSimple.Direction.FORWARD);  // 右臂電機方向設定為正轉
        armEnc = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "armR")));  // 初始化編碼器
        ArmPID = new PIDController(armP, armI, armD);  // 初始化 PID 控制器
    }

    /**
     * 讓手臂移動到目標角度。
     * 使用 PID 控制器計算輸出功率，並根據當前模式進行重力補償。
     */
    public void moveToAngle() {
        armPosNow = armEnc.getPositionAndVelocity().position / ARM_ENC_TO_DEG;  // 獲取當前角度位置
        double target = clamp(armTarget, ARM_BOTTOM_LIMIT, ARM_UP_LIMIT);  // 限制目標角度在範圍內

        // 設置 PID 參數
        double armkP = hangMode ? 0.1 : armP;  // 如果處於掛鉤模式，使用不同的 kP 值
        armF = hangMode ? 0 : Math.cos(Math.toRadians(armPosNow)) * slidePosNow * arm_f_coeff;  // 重力補償
        ArmPID.setPID(armkP, armI, armD);
        ArmPID.setTolerance(2);  // 容許誤差角度

        armOutput = ArmPID.calculate(armPosNow, target) + armF;  // 計算 PID 輸出功率

        armOutput = clamp(armOutput, ARM_POWER_MIN, ARM_POWER_MAX);  // 限制輸出功率在範圍內
        armL.setPower(armOutput);  // 設定左電機輸出功率
        armR.setPower(armOutput);  // 設定右電機輸出功率
    }

    /**
     * 停止手臂運行，將電機功率設為 0。
     */
    public void stop() {
        armL.setPower(0);
        armR.setPower(0);
    }

    /**
     * 設定當前滑軌位置角度，用於更新 `slidePosNow` 變數。
     *
     * @param slidePosNow 當前滑軌的位置角度
     */
    public void setSlidePosNow(double slidePosNow) {
        this.slidePosNow = slidePosNow;  // 直接將傳入的角度值設定為當前滑軌位置
    }

    /**
     * 取得當前手臂位置角度。
     *
     * @return 當前手臂位置角度
     */
    public double getArmPosNow() {
        return armPosNow;
    }

    /**
     * 取得當前設定的目標角度。
     *
     * @return 手臂的目標角度
     */
    public double getArmTarget() {
        return armTarget;
    }

    /**
     * 確認是否處於掛鉤模式。
     *
     * @return true 表示處於掛鉤模式，false 表示普通模式
     */
    public boolean isHangingMode() {
        return hangMode;
    }

    /**
     * 取得當前 PID 控制器的輸出功率。
     *
     * @return 當前 PID 控制器輸出的功率值
     */
    public double getArmOutput() {
        return armOutput;
    }

    /**
     * 設置手臂的目標角度。
     *
     * @param armTarget 新的手臂目標角度
     */
    public void setArmTarget(double armTarget) {
        this.armTarget = armTarget;
    }

    /**
     * 設置 PID 控制參數。
     *
     * @param p 比例參數 (P)
     * @param i 積分參數 (I)
     * @param d 微分參數 (D)
     */
    public void setPID(double p, double i, double d) {
        this.armP = p;
        this.armI = i;
        this.armD = d;
        ArmPID.setPID(p, i, d);
    }

    /**
     * 設置掛鉤模式開啟或關閉。
     *
     * @param hangMode true 表示進入掛鉤模式，false 表示關閉掛鉤模式
     */
    public void setHangMode(boolean hangMode) {
        this.hangMode = hangMode;
    }

    /**
     * 取得左、右電機的電流讀數 (單位：安培)。
     *
     * @return 一個包含兩個值的陣列，分別為左電機和右電機的電流讀數
     */
    public double[] getCurrent() {
        return new double[]{
                armL.getCurrent(CurrentUnit.AMPS),
                armR.getCurrent(CurrentUnit.AMPS)
        };
    }
}
