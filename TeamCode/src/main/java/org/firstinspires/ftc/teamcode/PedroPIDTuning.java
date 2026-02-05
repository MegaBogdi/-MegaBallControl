package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class PedroPIDTuning {
    public static double kP = 0.0;   //TranslationalPID
    public static double kI = 0.0;   //TranslationalPID
    public static double kD = 0.0;   //TranslationalPID
    public static double kF = 0.0;   //TranslationalPID

    public static double centripicalScaling=0.0005;
    public static double breakingStrenght =1;
    public static double breakingStart =1;

    public static double lX=0.0;
    public static double lY=0.0;
    public static double rX=0.0;


}
