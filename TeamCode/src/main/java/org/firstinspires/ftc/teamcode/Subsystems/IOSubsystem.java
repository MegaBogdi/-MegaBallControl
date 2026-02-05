package org.firstinspires.ftc.teamcode.Subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static  org.firstinspires.ftc.teamcode.Gains.SorterGains.*;
import static  org.firstinspires.ftc.teamcode.Gains.TurretGains.*;
import static  org.firstinspires.ftc.teamcode.Gains.RPMGains.*;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Gains;
import org.firstinspires.ftc.teamcode.TrapezoidProfile;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;


import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;


@Config
public class IOSubsystem extends SubsystemBase {

    private HardwareMap hmap;
    public CachingServo park;
    private final CachingCRServo turret;
    private final CachingServo push1;
    private final CachingServo push2;
    private final CachingServo hood;
    private final Limelight3A lime;
    public final CachingDcMotorEx sorter;
    private final CachingDcMotorEx collector;
    private final CachingDcMotorEx launcher1;
    private final CachingDcMotorEx launcher2;
    private final NormalizedColorSensor snsr1;


    public boolean teamIsRed=false;
    public static double testRPM=1500;
    public static double testAngle=0.3;
    public static double redX=130.5;
    public static double redY=125;
    public static double testPower;
    private int space = 2730;//2730;
    private final double[] BLUE_GOAL = {10.5,134};
    private final double[] RED_GOAL = {130.5,125};

    private final double tick_per_rotation = 8192;

    public static double RPM=0;
    public static double targetRPM=0;
    public static int targetPos;
    public static int currentTurret;
    public static double targetTurret = Math.toRadians(0);
    private long lastUpdate = 1;
    public int[] ALL = new int[3];
    public int[] MOTIF = new int[3];
    public int demand_index;
    public boolean nowOpen = false;
    public boolean demanding = false; // temportal latch for demanding
    public boolean climbing = false; // temportal latch for climb
    public double   SERVO_MIN_LIMIT = 0;
    public static double SERVO_MAX_LIMIT = 0.35;

    public double PUSH_MIN_LIMIT = 0.056;
    public double PUSH_MAX_LIMIT = 0.156;


    private PIDController pid;
    private PIDController pidT;
    private PIDController pidRPM;
    private final TrapezoidProfile sorter_profile;






    public IOSubsystem(final HardwareMap hMap) {
        snsr1 = hMap.get(NormalizedColorSensor.class, "senA");
        snsr1.setGain(8.0f);

        ALL[0] = 0;        MOTIF[0] = 1;
        ALL[1] = 0;        MOTIF[1] = 2;
        ALL[2] = 0;        MOTIF[2] = 1;

//===========================MOTOR==================================================

        collector = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "coll")));
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sorter = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "sort")));
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorter.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1 = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "lauA"))); // ENCODER RPM
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2 = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "lauB"))); // ENCODER TURRET
        //launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //==========================LED===================================

        //==========================SERVO=========================================

        push1 = new CachingServo(hMap.get(Servo.class,"pushA"));
        push2 = new CachingServo(hMap.get(Servo.class,"pushB"));
        push2.setDirection(Servo.Direction.REVERSE);
        push1.setPosition(PUSH_MIN_LIMIT);
        push2.setPosition(PUSH_MIN_LIMIT);

        hood =  new CachingServo(hMap.get(Servo.class,"hood"));
        hood.setDirection(Servo.Direction.REVERSE);
        hood.setPosition(SERVO_MIN_LIMIT);


        park = new CachingServo(hMap.get(Servo.class,"park"));
        park.setDirection(Servo.Direction.REVERSE);

        //=====================CRSERVO ======================================
        turret = new CachingCRServo(hMap.get(CRServo.class, "turet"));
        //turret.setPower(0);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);


        lime = hMap.get(Limelight3A.class, "limelight");
        lime.pipelineSwitch(0);
        //lime.start();



        //follower = Constants.createFollower(hMap);
        //follower.setStartingPose(new Pose(89.49532710280373, 9.196261682242984, Math.toRadians(270)));
        //follower.startTeleopDrive();
        //follower.update();

        pid = new PIDController(sP, sI, sD);
        targetPos = 0;

        pidT = new PIDController(Gains.TurretGains.tP,Gains.TurretGains.tI,Gains.TurretGains.tD);
        targetRPM=0;

        pidRPM = new PIDController(Gains.RPMGains.kP, Gains.RPMGains.kI, Gains.RPMGains.kD);
        targetRPM=0;


        sorter_profile = new TrapezoidProfile(space,0,0,false);

        //GoBildaPinpointDriver pinpoint;



        this.hmap = hMap;

    }

    public void setALIANCE(boolean ALIANCE){
        teamIsRed = ALIANCE;
    }


/*
    double[][] shootingData = {
            { 100, 2300, 0.8 },
            {  120,   2300,  0.6},
            { 150,   2460,  0.1 },  //2460
            { 164,   2570,  0.1 }, // 2570
            { 192,   2700,  0.1 },
            { 200,   2730,  0.1 },  //
            { 250,   2960,  0.1 },
            { 264,   3000,  0.1 },
            { 350,   3460,  0.1 },
    };*/

    double[][] shootingData = {
            {350,testRPM,testAngle},
            {400,testRPM,testAngle}
    };




    public double[] getInterpolatedValues(double currentDistance)
    {
        if (currentDistance == -1) return new double[] {2300, 0};

        if (currentDistance <= shootingData[0][0]) {
            return new double[]{ shootingData[0][1], shootingData[0][2] };
        }

        if (currentDistance >= shootingData[shootingData.length - 1][0]) {
            int last = shootingData.length - 1;
            return new double[]{ shootingData[last][1], shootingData[last][2] };
        }


        for (int i = 0; i < shootingData.length - 1; i++) {
            double distLow = shootingData[i][0];
            double distHigh = shootingData[i+1][0];

            if (currentDistance > distLow && currentDistance < distHigh) {

                double t = (currentDistance - distLow) / (distHigh - distLow);

                double rpm = shootingData[i][1] + (shootingData[i+1][1] - shootingData[i][1]) * t;
                double angle = shootingData[i][2] + (shootingData[i+1][2] - shootingData[i][2]) * t;

                return new double[]{ rpm, angle };
            }
        }
        return new double[]{0, 0};
    }


    public int getMotif(){
        LLResult result = lime.getLatestResult();

        if (result == null || !result.isValid()) return -1;
        else {
            return result.getFiducialResults().stream()
                    .map(LLResultTypes.FiducialResult::getFiducialId)
                    .filter(id -> id == 21 || id == 22 || id == 23)
                    .findFirst()
                    .orElse(-1);

        }
    }
    public void startLimeLight(){
        lime.start();
    }
    public void pauseLimeLight(){
        lime.pause();
    }
    public void stopLimeLight(){
        lime.stop();
    }


    public void motifTranslate(int id){
        if (id==-1){MOTIF[0]=2;MOTIF[1]=1;MOTIF[2]=1;}
        else if (id==21){MOTIF[0]=2;MOTIF[1]=1;MOTIF[2]=1;}
        else if (id==22){MOTIF[0]=1;MOTIF[1]=2;MOTIF[2]=1;}
        else if (id==23){MOTIF[0]=1;MOTIF[1]=1;MOTIF[2]=2;}
    }

    public boolean checkMotifFind(int id){
        if (id==-1){return false;}
        else{return true;}
    }



    public double getDistanceOdom(Pose curPose){
        double x = curPose.getX();
        double y = curPose.getY();

        double dist;
        if (teamIsRed){
            double Ry = RED_GOAL[1] - y;
            double Rx = 145 - x;
            dist = Math.sqrt(Math.pow(Rx,2) + Math.pow(Ry,2));
        } else {
            double Ry = BLUE_GOAL[1] - y;
            dist = Math.sqrt(Math.pow(x,2) + Math.pow(Ry,2));
        }
        return dist*2.54;
    }

// RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM


    public void setMotorRPM(double rpm){
        targetRPM = rpm;
    }
    public double returnRPM(){
        return RPM;
    }
    public double returnTargetRPM(){
        return targetRPM;
    }
    public double returnTargetPos(){
        return targetPos;
    }
    public double returnSorterPos(){
        return sorter.getCurrentPosition();
    }
    public void setLauncherPower(double power){
        launcher1.setPower(power);
        launcher2.setPower(power);
    }

// ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL
    public void setNormalizedAngle(double rawAngle) {

        rawAngle = clamp(rawAngle, SERVO_MIN_LIMIT, SERVO_MAX_LIMIT);

        hood.setPosition((rawAngle - SERVO_MIN_LIMIT) / (SERVO_MAX_LIMIT - SERVO_MIN_LIMIT));
    }

    public void setTargetTurretRads(double newTargetPos){
        targetTurret = newTargetPos;
    }


    public double getAngle(Pose pose){
        RED_GOAL[0 ]=redX; RED_GOAL[1]=redY;
        double Alpha;
        if (teamIsRed){
            Alpha = Math.atan((RED_GOAL[1]-pose.getY())/(RED_GOAL[0]-pose.getX()));
            Alpha -= Math.toRadians(0);

        } else {
            Alpha = Math.atan((BLUE_GOAL[1] - pose.getY()) /(pose.getX()-BLUE_GOAL[0]));
            Alpha = -Alpha;
            Alpha += Math.toRadians(180);
        }
        return Alpha;
    }


    public void setHood(double ang)
    {
        hood.setPosition(ang);

    }

// Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret


    public double[] returnTuret(){
        return new double[] {launcher1.getCurrentPosition(), launcher1.getVelocity()};

    }
    public double returnTargetTuret(String mode){
        if(mode=="rads"){return targetTurret;} else {return rads2ticks(targetTurret);}
    }

    public boolean isTurretReady(double alpha,Follower recivedFollower){
        if (Math.abs(AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(recivedFollower.getPose().getHeading() + Math.toRadians(180)) + tick2rads(currentTurret) - alpha))<1){return true;}else{return false;}
    }

    public double testTurretReady(double alpha,Follower recivedFollower) {
        double heading = AngleUnit.normalizeRadians(recivedFollower.getPose().getHeading() + Math.toRadians(180));
        double turretRad = tick2rads(currentTurret);
        double absTurretHeading = turretRad + heading;
        return Math.abs(AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(recivedFollower.getPose().getHeading() + Math.toRadians(180)) + tick2rads(currentTurret) - alpha));
    }
    public int rads2ticks(double rads){
        double ratio = 207.0/44.0;
        return (int)Math.round((rads/(Math.PI*2)) * 8192 * ratio);
    }
    public double tick2rads(int tick){
        double ratio = 207.0/44.0;
        return tick * (Math.PI * 2) / ratio / 8192.0;
    }

    public int detectDominant(NormalizedRGBA c) {
        // Grab raw normalized values (0..1)
        float r = c.red;
        float g = c.green;
        float b = c.blue;

        // Optional: throw away very dim readings
        float sum = r + g + b;
        if (sum < 0.01f) return 0; // too dark / no object

        // Normalize so brightness doesn’t matter
        r /= sum;
        g /= sum;
        b /= sum;

        // Dominance thresholds
        if (r > g * 1.3f && r > b * 1.3f) {
            return 0;
        } else if (g > r * 2.0f && g > b * 1.2 && g>0.007) {
            return 2;
        } else if (b > r * 1.01f && b > g * 1.01f) {
            return 1;
        } else {
            return 0; // nothing clearly dominant
        }
    }

    public float[] getRGB() {
        NormalizedRGBA snsr = snsr1.getNormalizedColors();
        // Grab raw normalized values (0..1)
        float r = snsr.red;
        float g = snsr.green;
        float b = snsr.blue;

        return new float[] {r,g,b};

    }

    public boolean isOcupied(){
        if (detectDominant(snsr1.getNormalizedColors())==0){
            return false;}
        else{
            return true;}
    }
    public int readSensor() {
        int rez = detectDominant(snsr1.getNormalizedColors());
        return rez;
    }
    public void climb(){
        targetPos += space;
        cycle_up();
    }
    public void climbDown(){
        targetPos -= space;
        cycle_up();cycle_up();
    }

    public void start_intake(){collector.setPower(1);}
    public void stop_intake(){collector.setPower(0);}
    public void expel_intake(){collector.setPower(-1);}
    double TICKS_PER_REVOLUTION = 28.0;
    public double TPS2RPM(double tps){
        RPM = (tps/TICKS_PER_REVOLUTION)*60;
        return RPM;
    }

    public boolean isRPMready(){
        if (Math.abs(targetRPM-RPM)<RPM_EPS+20){return true;}
        else {return false;}
    }

    public void setPush(double pos){
        push1.setPosition(pos);
        push2.setPosition(pos);
    }
    public double getPush(){
        return push1.getPosition();}

    public int getTicksSort(){
        return sorter.getCurrentPosition();
    }

    public int ocupied(){
        int count = 0;
        for(int ball:ALL){
            if (ball != 0){
            count += 1;}
        }
        return count;
    }

    public void cycle_up(){
        int temp = ALL[2];
        ALL[2] = ALL[1];
        ALL[1] = ALL[0];
        ALL[0] = temp;
    }

    public void regist(){
        ALL[0] = readSensor();
    }
    public void regist_release(){
        ALL[1] = 0;
        demand_index += 1;
    }

    public void close(){
        if (nowOpen){targetPos+=space/2;}
        nowOpen = false;
    }
    public void open(){
        if (!nowOpen){targetPos-=space/2;}
        nowOpen = true;
    }
    public void climb1inDir(){
        int dir = 1;
        double error = Math.abs(targetPos-sorter.getCurrentPosition());
        targetPos += space;
        double try1error = Math.abs(targetPos-sorter.getCurrentPosition());
        if (try1error>error){dir = -dir;}
        targetPos -= space; //revert try one;
        if (dir>0){climb();}else{climbDown();}
    }
    public boolean isSorterReady(){
        if (Math.abs(sorter.getCurrentPosition()-targetPos)< SORT_POS_EPS +40){
            return true;
        } else {return false;}

    }
    public void rectify(){
        targetPos +=space/2;
    }



    public boolean getDemanded(){
        int demand = MOTIF[demand_index%3];
        if (ALL[1]==demand){
            return true;
        } else if (ALL[2]==demand){
            targetPos -= space;
            cycle_up();
            cycle_up();
            return true;
        } else if (ALL[0]==demand){
            targetPos +=space;
            cycle_up();
            return true;
        } else {return false;}
    }






    public void update_sep_pid(){
        pid.setPID(sP,sI, sD);

        double currentPos = sorter.getCurrentPosition();
        double currentVel = sorter.getVelocity();
        double error  = targetPos-currentPos;

        double output = pid.calculate(currentPos,targetPos);

        if (Math.abs(currentVel) < SORT_VEL_EPS){ // if static
            output+= sS *Math.signum(error);
        } else if (Math.abs(error) > SORT_POS_EPS){ // if should be moving
            output+= sT*Math.signum(error);
        }




        if (Math.abs(error)< SORT_POS_EPS){output=0;} //TOLERANCE  on error epsilon
        output = Math.max(-1,Math.min(output,1));      //CLAMP POWER
        sorter.setPower(output);
    }



    public void update_turret_pid(){
        pidT.setPID(tP,tI,tD);
        currentTurret =  launcher1.getCurrentPosition();
        double targetTurretTicks = Math.max(MIN_TICKS,Math.min(rads2ticks(targetTurret), MAX_TICKS)); // clamp before it gets out of range
        double currentTurretVel = launcher1.getVelocity();
        double error = targetTurretTicks-currentTurret;

        double output = pidT.calculate(currentTurret,targetTurretTicks);

        if (Math.abs(currentTurretVel) < TUR_VEL_EPS){
            output += tS*Math.signum(error);
        } else if (Math.abs(error) > TUR_POS_EPS){
            output += tT*Math.signum(error);
        }

        if (Math.abs(error) < TUR_POS_EPS){
            output=0;
        }

        if (currentTurret > MAX_TICKS){if(output>0){output=0;}} //targetTurret= tick2rads(MIN_TICKS);} //CLAMP POWER IF IT GETS OUT OF BOUNDS but let it only exit
        if (currentTurret < MIN_TICKS){if(output<0){output=0;}} //targetTurret = tick2rads(MAX_TICKS);}

        output = Math.max(-1,Math.min(output,1));                   //CLAMP POWER

        turret.setPower(output);
    }
    public void testTS(){
        turret.setPower(tS);
    }


    public void update_RPM_pid(){
        pidRPM.setPID(kP,kI,kD);
        RPM = TPS2RPM(launcher2.getVelocity());
        double error = targetRPM - RPM;

        double output = kS*Math.signum(targetRPM) + kV*targetRPM;

        output += pidRPM.calculate(RPM,targetRPM);

        if (targetRPM==0){output=0;}
        output = Math.max(-1,Math.min(output,1));


        setLauncherPower(output);
    }

    public void testThePower(double power){
        setLauncherPower(power);
    }

    public void update_sorter_profile(double t){
        double vel = sorter.getVelocity();
        double pos = sorter.getCurrentPosition();
        double accCmd = sorter_profile.accel(t);
        double velCmd = sorter_profile.velocity(t);
        double posCmd = sorter_profile.position(t);

        double fb = sorter_profile.feedBack(velCmd,posCmd,vel,pos);
        double ff = sorter_profile.feedForward(velCmd,accCmd,vel);
        double output = fb+ff;
        output = Math.min(1,Math.max(-1,output));

        // sorter.setPower(output);

    }

// 207 gear mare
    //90 gear servo
    //44 gear encoder


}


