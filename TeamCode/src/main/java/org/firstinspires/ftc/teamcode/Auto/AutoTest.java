package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.concurrent.atomic.AtomicBoolean;
@Disabled
@Config
@Autonomous(name = "AutonomieTest", group = "Auto")
public class AutoTest extends CommandOpMode {

    private Pose startPose = new Pose(24.224,126.953,Math.toRadians(136));
    private Pose shootPos = new Pose(58.542,84.336,Math.toRadians(136));
    private Pose shootFarPos =  new Pose(60.5607476635514,11.214953271028039,Math.toRadians(180));
    private Pose colectSpike1Pos = new Pose(16.373831775700936,83.66355140186916,Math.toRadians(180));
    private Pose colectSpike2Pos = new Pose(9.869158878504672,60.33644859813084,Math.toRadians(180));
    private Pose helpSpike2Pos = new Pose(62.35514018691589,56.299065420560744,Math.toRadians(180));
    private Pose colectSpike3Pos = new Pose(9.869158878504672,36.560747663551396,Math.toRadians(180));
    private Pose helpSpike3Pos = new Pose(73.1214953271028,29.831775700934557,Math.toRadians(180));
    private Pose pullPos = new Pose(16.373831775700936,69.98130841121497 ,Math.toRadians(180));
    private Pose helpPullPos = new Pose(32.299065420560744,62.3551401869159 ,Math.toRadians(180));
    private Pose helpPullReturnPos = new Pose(48.44859813084112,70.20560747663552 ,Math.toRadians(180));

    private Pose parkPos = new Pose(38.13084112149532,36.560747663551396,Math.toRadians(180));


    private Telemetry telemetryA;
    private IOSubsystem IO;
    //private DriveSubsystem chassis;
    private Follower follower = Constants.createFollower(hardwareMap);

    double[] interpValues;
    double dist;
    int rez;

    private boolean succesDemand;

    AtomicBoolean stopIntakeBool = new AtomicBoolean(false);

    public enum PathState {
        START2SHOOT,
        READMOTIF,
        SHOOT,
        SHOOT2SPIKE2,
        SPIKE2TOLEVER,
        LEVER2SHOOT,
        SHOOT2SPIKE1,
        SPIKE1TOSHOOT,
        SHOOT2SPIKE3,
        SPIKE3TOSHOOTFAR,
        SHOOTFAR2PARK,


        END
    }

    PathState pathState;

    PathState lastState;

    private PathChain StartToShootPath, ShootToSpike2Path, Spike2ToLeverPath, LeverToShootPath, ShootToSpike1Path,Spike1ToShootPath,ShootToSpike3Path, Spike3ToShootFarPath,ShootFarToParkPose;



    public void buildPaths(){
    }

    private Command Motify(){return new SequentialCommandGroup(
      new WaitUntilCommand(()->{rez = IO.getMotif();return IO.checkMotifFind(rez);}),
      new InstantCommand(()->IO.motifTranslate(rez))
    );}

    private Command stopIntake(){return new InstantCommand(()->{IO.stop_intake();IO.close();});}
    private Command Intake(){
        return new ParallelRaceGroup(
            new ParallelDeadlineGroup(
                    new WaitUntilCommand(()-> IO.ocupied() < 3).withTimeout(2000),// REPLACE WITH REAL VALUE!!!!!!!!!!!!!!
                    new ParallelCommandGroup(
                            new InstantCommand(()->IO.start_intake()),
                            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                            new InstantCommand(()->IO.regist()),
                                            new InstantCommand(()->IO.climb()),
                                            new WaitUntilCommand(()->!IO.isOcupied()),
                                            new InstantCommand()
                                    ),
                                    new InstantCommand(),
                                    ()->IO.isOcupied() // returns boolean.
                            )
                    ).perpetually()
            ),
            new WaitUntilCommand(stopIntakeBool::get)
        ).andThen(new SequentialCommandGroup(stopIntake(),new InstantCommand(()->stopIntakeBool.set(false))));}


    private Command quickPush(){ return new SequentialCommandGroup(
            new InstantCommand(()->IO.setPush(0.6)),
            new WaitCommand(200),
            new InstantCommand(()->IO.setPush(0.25)),
            new WaitCommand(200),
            new InstantCommand(()->IO.regist_release())
    );}
    private Command Demand(){
        return new ParallelDeadlineGroup(
            new WaitUntilCommand(()->IO.ocupied()==0),
            new SequentialCommandGroup(
                    new InstantCommand(()-> IO.getDemanded(),IO),
                    new WaitCommand(1000),
                    quickPush()
            ).perpetually(),
            new RunCommand(()->IO.start_intake())
    );}
    private Command demand (){
    return new SequentialCommandGroup(
        new InstantCommand(()-> succesDemand = IO.getDemanded()),
        new ConditionalCommand(
            new SequentialCommandGroup(
            new WaitUntilCommand(()->IO.isSorterReady()&&IO.isRPMready()),
            quickPush()
            ),
            new InstantCommand(()->IO.demand_index+=1),
            ()->succesDemand
        )
    );}

    private Command interp(){
        return new RunCommand(()->{
            dist = IO.getDistanceOdom(follower.getPose());
            interpValues = IO.getInterpolatedValues(dist);
            IO.setHood(interpValues[1]);
            IO.setMotorRPM(interpValues[0]);
        });
    }


    public void pathStateUpdate(){
        switch(pathState){
            case START2SHOOT:
                follower.followPath(StartToShootPath);
                pathState = PathState.READMOTIF;
                lastState = PathState.START2SHOOT;
                break;
            case READMOTIF:
                if (!follower.isBusy()){

                }
            case SHOOT:
                if (!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new ParallelDeadlineGroup(
                                    new WaitCommand(2000),  //REPLACE WITH REAL VALUE !!!!!!!!!!!!!!!!!!!!!!!!!!!! timeout
                                    Demand(),
                                    interp()
                            ),
                            new InstantCommand(()->{
                                if (lastState==PathState.START2SHOOT){
                                    pathState = pathState.SHOOT2SPIKE2;
                                    follower.followPath(ShootToSpike2Path);
                                } else if (lastState==PathState.LEVER2SHOOT){
                                    pathState = PathState.SHOOT2SPIKE1;
                                    follower.followPath(ShootToSpike1Path);
                                } else if (lastState==PathState.SPIKE1TOSHOOT){
                                    pathState = PathState.SHOOT2SPIKE3;
                                    follower.followPath(ShootToSpike3Path);
                                }
                            }),
                            Intake()
                        )
                    );
                }
                break;
            case SHOOT2SPIKE2:
                if (!follower.isBusy()){
                    stopIntakeBool.set(true);
                    pathState = PathState.SPIKE2TOLEVER;
                    follower.followPath(Spike2ToLeverPath);
                }
                break;
            case SPIKE2TOLEVER:
                if(!follower.isBusy()){
                    pathState = PathState.LEVER2SHOOT;
                    follower.followPath(Spike2ToLeverPath);
                }break;
            case LEVER2SHOOT:
                if(!follower.isBusy()){
                    pathState = PathState.SHOOT;
                    lastState = PathState.LEVER2SHOOT;
                    follower.followPath(LeverToShootPath);
                }break;
            case SHOOT2SPIKE1:
                if(!follower.isBusy()){
                    stopIntakeBool.set(true);
                    pathState = PathState.SPIKE1TOSHOOT;
                    follower.followPath(ShootToSpike1Path);
                }break;
            case SPIKE1TOSHOOT:
                if(!follower.isBusy()){
                    pathState = PathState.SHOOT;
                    lastState= PathState.SPIKE1TOSHOOT;
                    follower.followPath(Spike1ToShootPath);
                }break;
            case SHOOT2SPIKE3:
                if(!follower.isBusy()){
                    stopIntakeBool.set(true);
                    pathState = PathState.SPIKE3TOSHOOTFAR;
                    follower.followPath(Spike3ToShootFarPath);
                }break;
            case SPIKE3TOSHOOTFAR:
                if(!follower.isBusy()){
                    pathState = PathState.SHOOTFAR2PARK;
                    follower.followPath(ShootFarToParkPose);
                }break;


            default:
                telemetryA.addLine("defaulted");


        }
    }


    @Override
    public void initialize(){

        IO = new IOSubsystem(hardwareMap);
        //chassis = new DriveSubsystem(hardwareMap);
        follower.setStartingPose(startPose);

        pathState = PathState.START2SHOOT;


        StartToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPos.getHeading())
                .build();

        ShootToSpike2Path = follower.pathBuilder()
                .addPath(new BezierCurve(shootPos,helpSpike2Pos,colectSpike2Pos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();

        Spike2ToLeverPath = follower.pathBuilder()
                .addPath(new BezierCurve(colectSpike2Pos,helpPullPos,pullPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();

        LeverToShootPath = follower.pathBuilder()
                .addPath(new BezierCurve(pullPos,helpPullReturnPos,shootPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();

        ShootToSpike1Path = follower.pathBuilder()
                .addPath(new BezierLine(shootPos,colectSpike1Pos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();
        Spike1ToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(colectSpike1Pos,shootPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();
        ShootToSpike3Path= follower.pathBuilder()
                .addPath(new BezierCurve(shootPos,helpSpike3Pos,colectSpike3Pos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();
        Spike3ToShootFarPath = follower.pathBuilder()
                .addPath(new BezierLine(colectSpike3Pos,shootFarPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();
        ShootFarToParkPose = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPos,parkPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();




        follower.setStartingPose(startPose);
    }

    @Override
    public void run(){
        follower.update();
        pathStateUpdate();
        telemetryA.addData("pos","x=%.3f | y=%.3f | heading=%.3f",follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading());
        telemetryA.addData("followerBussy=",follower.isBusy());

    }


}
