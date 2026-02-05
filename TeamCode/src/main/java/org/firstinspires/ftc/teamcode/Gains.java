package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;


public class Gains {
    @Config
    public static class ProfileGains {
        public static double kP = 0;
        public static double kD = 0;

        public static double kJ = 0;
        public static double velCmdEps;
    }
    @Config
    public static class PIDFGains{
        public static double kP = 0;
        public static double kD = 0;

    }

    @Config
    public static class SorterGains{
        // Gains SORTER
        public static double sP =0.0006;  //0.000405
        public static double sI =0.0;   //0.00455
        public static double sD =0.000038;  //0.0000230.000085
        public static double sS = 0.0436 ;
        public static double sT = 0.068;
        public static double SORT_POS_EPS =20;
        public static double SORT_VEL_EPS =2;
        public static double minVelCmd; // above this we are oficialy trying to move
        public static double jamVel; // above this we are not moving
        public static double pwrMin; // above this we are trying HARD
        public static double jamTime; // above this its stuck for to long
    }

    @Config
    public static class TurretGains{
        public static double tP=0.00022;
        public static double tI=0;
        public static double tD=0.000014;
        public static double tS=0.04;
        public static double tT=0.095;
        public static double TUR_POS_EPS =50;
        public static double TUR_VEL_EPS =5000;
        public static int MAX_TICKS =12000;
        public static int MIN_TICKS =-15000;

    }
    @Config
    public static class RPMGains{
        public static double kP =0.007;
        public static double kI =0.0;
        public static double kD =0.0;  // try to keep 0
        public static double kS = 0.151;   //Static power;
        public static  double kV=0.000185;  // pure feedForward
        public static double RPM_EPS = 10; // RPM TOLERANCE
    }


}