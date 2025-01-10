package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * WristSubsystem 負責控制手腕機構的子系統。
 * 通過控制伺服馬達的位置，實現手腕的升降和旋轉。
 */
@Config
public class WristSubsystem extends SubsystemBase {
    protected Servo FrontL, FrontR;  // 左、右手腕伺服馬達
    protected AnalogInput FrontL_Pos, FrontR_Pos;  // 左、右手腕位置感測器

    // 系統參數
    public static double gear_ratio = 2.888888;  // 齒輪比
    public static double currentAngleRight;  // 右側當前角度
    public static double tarAngleLeft = 0;  // 左側目標角度
    public static double tarAngleRight = 0; // 右側目標角度
    public static double lift_Offset = -10;  // 升降角度偏移量
    public static double turn_Offset = -15;  // 旋轉角度偏移量
    public static double lift_Max = 90;  // 升降最大角度
    public static double lift_Mini = -90;  // 升降最小角度
    public static double turn_Max = 90;  // 旋轉最大角度
    public static double turn_Mini = -90;  // 旋轉最小角度
    public static double maxServoAngleLeft = 270.078;  // 左伺服馬達最大角度
    public static double maxServoAngleRight = 270.078;  // 右伺服馬達最大角度
    public static double tarPositionLeft;  // 左伺服馬達目標位置 (比例)
    public static double tarPositionRight;  // 右伺服馬達目標位置 (比例)

    /**
     * WristSubsystem 建構子。
     * 初始化伺服馬達和位置感測器。
     *
     * @param hardwareMap 硬體映射，用於獲取硬體元件
     */
    public WristSubsystem(HardwareMap hardwareMap) {
        FrontL = hardwareMap.get(Servo.class, "frontl");  // 左側伺服馬達
        FrontR = hardwareMap.get(Servo.class, "frontr");  // 右側伺服馬達
        FrontL_Pos = hardwareMap.get(AnalogInput.class, "lpos");  // 左側位置感測器
        FrontR_Pos = hardwareMap.get(AnalogInput.class, "rpos");  // 右側位置感測器
    }

    /**
     * 控制手腕位置，根據目標升降角度和旋轉角度設定伺服馬達的位置。
     * 會根據齒輪比和限制條件進行計算。
     *
     * @param liftAng 目標升降角度
     * @param turnAng 目標旋轉角度
     */
    public void wristToPosition(double liftAng, double turnAng) {
        turnAng /= gear_ratio;  // 考慮齒輪比
        turnAng = clamp(turnAng, turn_Mini, turn_Max);  // 限制旋轉角度
        liftAng = clamp(liftAng, lift_Mini, lift_Max);  // 限制升降角度

        turnAng -= turn_Offset;  // 套用旋轉偏移量
        liftAng -= lift_Offset;  // 套用升降偏移量
        tarAngleLeft = liftAng + (maxServoAngleLeft / 2);  // 左側目標角度計算
        tarAngleRight = liftAng - turnAng + (maxServoAngleRight / 2);  // 右側目標角度計算
        tarPositionLeft = tarAngleLeft / maxServoAngleLeft;  // 左側伺服馬達目標位置 (比例)
        tarPositionRight = tarAngleRight / maxServoAngleRight;  // 右側伺服馬達目標位置 (比例)

        FrontL.setPosition(1 - tarPositionLeft);  // 設定左伺服馬達位置
        FrontR.setPosition(tarPositionRight);  // 設定右伺服馬達位置
    }

    /**
     * 取得升降最大角度。
     *
     * @return 升降的最大角度
     */
    public double getLiftMax() {
        return lift_Max;
    }

    /**
     * 取得升降最小角度。
     *
     * @return 升降的最小角度
     */
    public double getLiftMin() {
        return lift_Mini;
    }

    /**
     * 取得旋轉最大角度。
     *
     * @return 旋轉的最大角度
     */
    public double getTurnMax() {
        return turn_Max;
    }

    /**
     * 取得旋轉最小角度。
     *
     * @return 旋轉的最小角度
     */
    public double getTurnMin() {
        return turn_Mini;
    }
}
