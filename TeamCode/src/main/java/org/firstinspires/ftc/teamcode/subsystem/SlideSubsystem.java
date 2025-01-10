package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class SlideSubsystem {
    private DcMotorEx slide;
    private PIDController slidePID;

    // PID 參數
    public static double slideP = 0.5;
    public static double slideI = 0.0;
    public static double slideD = 0.0;
    public static double slideF = 0.0;
    public static double slideP_hang = 1.5;

    // 馬達參數與轉換係數
    public static double slide_motorEnc = 103.8; // 每轉的編碼器刻度
    public static double slide_Ratio = 1 / 1.4; // 齒輪減速比
    public static double slide2length = slide_motorEnc * slide_Ratio / 0.8; // 將編碼器刻度轉換為滑軌長度

    // 長度限制參數
    private double smax = 98.0; // 最大伸展長度
    private double smax0 = 76.0; // 手臂角度在 0 度時的最大長度
    private double smin = 40.0; // 最小縮回長度

    // 當前狀態變數
    private double slidePosNow = 0.0;
    private double slideTarget = 0.0;
    private double slidePower = 0.0;
    private static final double SLIDE_POWER_MAX = 1.0; // 最大功率限制
    private static final double SLIDE_POWER_MIN = -1.0; // 最小功率限制
    private boolean hangMode = false; // 是否處於掛鉤模式

    /**
     * 構造函數 - 初始化滑軌子系統
     */
    public SlideSubsystem(HardwareMap hardwareMap) {
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slidePID = new PIDController(slideP, slideI, slideD);
    }

    /**
     * 讓滑軌移動到指定位置
     * @param armPosNow 當前手臂角度位置 (度數)
     */
    public void moveToPosition( double armPosNow) {
        slidePosNow = (slide.getCurrentPosition() / slide2length) * 2 + smin; // 獲取滑軌當前長度

        // 根據手臂角度限制滑軌的最大長度
        double maxLimit = (armPosNow < 60) ? smax0 : smax;
        slideTarget = clamp(slideTarget, smin, maxLimit);

        // 選擇 PID 參數
        double slideKp = hangMode ? slideP_hang : slideP;
        slidePID.setPID(slideKp, slideI, slideD);

        // 計算功率輸出
        slidePower = slidePID.calculate(slidePosNow, slideTarget) + slideF;
        slidePower = clamp(slidePower, SLIDE_POWER_MIN, SLIDE_POWER_MAX);

        slide.setPower(slidePower);
    }

    /**
     * 停止滑軌運行
     */
    public void stop() {
        slide.setPower(0);
    }

    /**
     * 重置編碼器
     */
    public void resetEncoder() {
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * 獲取滑軌的當前長度 (cm)
     * @return 滑軌長度
     */
    public double getSlidePosNow() {
        slidePosNow = (slide.getCurrentPosition() / slide2length) * 2 + smin;
        return slidePosNow;
    }

    /**
     * 設置滑軌的目標位置 (cm)
     * @param target 目標位置
     */
    public void setTarget(double target) {
        this.slideTarget = target;
    }

    /**
     * 設置 PID 控制參數
     * @param p 比例增益
     * @param i 積分增益
     * @param d 微分增益
     */
    public void setPID(double p, double i, double d) {
        slideP = p;
        slideI = i;
        slideD = d;
        slidePID.setPID(p, i, d);
    }

    /**
     * 檢查是否到達目標位置
     * @param tolerance 容許誤差 (cm)
     * @return 如果在容許範圍內則返回 true
     */
    public boolean isAtTarget(double tolerance) {
        return Math.abs(slidePosNow - slideTarget) <= tolerance;
    }

    /**
     * 啟用或禁用掛鉤模式
     * @param hangMode 是否啟用掛鉤模式
     */
    public void setHangMode(boolean hangMode) {
        this.hangMode = hangMode;
    }

    /**
     * 獲取滑軌的目標位置
     * @return 目標位置 (cm)
     */
    public double getSlideTarget() {
        return slideTarget;
    }

    /**
     * 獲取滑軌當前電流值
     * @return 電流值 (安培)
     */
    public double getCurrent() {
        return slide.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * 設置滑軌長度限制
     * @param max 最大長度
     * @param min 最小長度
     * @param maxArm0 手臂角度 0 時的最大長度
     */
    public void setSlideLengthLimit(double max, double min, double maxArm0) {
        this.smax = max;
        this.smin = min;
        this.smax0 = maxArm0;
    }

    /**
     * 獲取滑軌最大長度
     * @return 最大長度 (cm)
     */
    public double getSmax() {
        return smax;
    }

    /**
     * 獲取滑軌最小長度
     * @return 最小長度 (cm)
     */
    public double getSmin() {
        return smin;
    }

    /**
     * 獲取滑軌角度 0 時的最大長度
     * @return 最大長度 (cm)
     */
    public double getSmax0() {
        return smax0;
    }
}
