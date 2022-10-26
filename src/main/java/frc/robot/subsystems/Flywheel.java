package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.util.spi.ToolProvider;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

// bottom rmp vs read dist.
//   close 
// 450 at 11.5
// 475 at 50
// 490 at 86
//    mid 
// 420 at 105
// 460 at 138
// 475 at 161
// 540 at 186
//    far 
// 552.5 at 216
// 580 at 252
// 625 at 285 
//https://www.desmos.com/calculator/l0sswpf3m6





/** Shooter flywheel subsystem */
public class Flywheel {

    private static final int FALCON_1 = 8;
    private static final int FALCON_2 = 9;
    private static final int ENC_TOP_A = 6;
    private static final int ENC_TOP_B = 7;
    private static final int ENC_BOTTOM_A = 4;
    private static final int ENC_BOTTOM_B = 5;

    // the max amount you can deviate from the desired speed before the driver station shows a red box
    private final double MAX_SPEED_DIFFERENCE = 0.05;

    TalonFX topMotor, bottomMotor;
    Encoder topEnc, bottomEnc;

    private int cpr;
    private double kP_top;
    private double kI_top;
    private double kD_top;

    private double kP_bottom;
    private double kI_bottom;
    private double kD_bottom;

    boolean useInternalEncoders = false;

    SimpleMotorFeedforward feedforward_top;
    SimpleMotorFeedforward feedforward_bottom;

    private double kS_top = 1.3116;  //Good
    private double kV_top = 0.39299;
    private double kA_top = 7.9824;

    private double kS_bottom = 1.519;
    private double kV_bottom = 0.45238;
    private double kA_bottom = 3.6465;

    private double desiredSpeedTop = 0;
    private double desiredSpeedBottom = 0;

    public double bottomWheelSpeed = 0;
    public double topWheelSpeed = 0;

    

    double topV = 0;
    double bottomV = 0;

    public double constant = 0;

    //For Sim
    DCMotor drive;
    EncoderSim encSim;
    FlywheelSim sim;

    /**
     * @param Robot robot instance 
     * @param useInternalEncoders true if Falcon built in encoders are being used
     */
    public Flywheel(Robot robot, boolean useInternalEncoders) {
        this.useInternalEncoders = useInternalEncoders;
        cpr = useInternalEncoders ? 2048 : 8192;


        //PID top    internalEncoder : REVCoder
        kP_top = useInternalEncoders ? 0 : 0;
        kI_top = useInternalEncoders ? 0 : 0;
        kD_top = useInternalEncoders ? 0 : 0;

        //PID bottom   internalEncoder : REVCoder
        kP_bottom = useInternalEncoders ? 0 : 0;
        kI_bottom = useInternalEncoders ? 0 : 0;
        kD_bottom = useInternalEncoders ? 0 : 0;

        feedforward_top = new SimpleMotorFeedforward(kS_top, kV_top, kA_top);
        feedforward_bottom = new SimpleMotorFeedforward(kS_bottom, kV_bottom, kA_bottom);

        //Hardware declarations
        topMotor = new TalonFX(FALCON_1);
        bottomMotor = new TalonFX(FALCON_2);

        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        
        if(!useInternalEncoders) {
            topEnc = new Encoder(ENC_TOP_A, ENC_TOP_B, false, EncodingType.k1X);
            bottomEnc = new Encoder(ENC_BOTTOM_A, ENC_BOTTOM_B, false, EncodingType.k1X);
            topEnc.setDistancePerPulse(-1.0 / 8192.0);
            bottomEnc.setDistancePerPulse(1.0 / 8192.0);
        }

        //For SIM
        if(!RobotBase.isReal()) {
            drive = DCMotor.getFalcon500(2);
            sim = new FlywheelSim(
                drive,      //Input Gearbox
                1,          //Gearing
                0.0023      //Moment of inertia
                );
        }

        robot.getScheduler().schedule(() -> {
            while (true)
                SmartDashboard.putBoolean("Shooter Ready", atDesiredSpeed());
        });
    }

    //For Update
    double topLastPos = 0;
    double bottomLastPos = 0;
    double lastTime = System.currentTimeMillis();

    //Conversions between RPM and TalonFX velocity units
    private double rpmToVelUnits(double rpm) {
        return (rpm / 600) * cpr;
    }

   
    /**
     * 
     * @param distance distance form the Limelight (inches)
     * @return A double Array with [0] as the Top flywheel Speeds; [1] as the Bottom Flywheel Speeds
     */
    public double[] clacRpms(double distance){
    double RpmArrays[] = new double[2]; 
    double closeShot[] = new double[2];
    closeShot[0] = 600;
    closeShot[1] = 490;

    if(distance <=100){RpmArrays[0] = 600; RpmArrays[1]= (395 + (21.5 * Math.log(distance)));}
    else if(distance >100 && distance <200){ RpmArrays[0] = 750; RpmArrays[1]= (496+(-1.89*distance)+(0.0114*Math.pow(distance, 2)));}
    else if(distance >= 200){ RpmArrays[0] = 900; RpmArrays[1]= (861+(-3.3*distance)+(0.00869*Math.pow(distance, 2)));} 
    else return closeShot;
    RpmArrays[1] += constant;
    return RpmArrays;

    }
    
    

    /**
     * Upates the shooter RPM based on the distance in inches from the goal
     * @param distance distance 
     * @param setManually choose to set bottom Rpm manually 
     * @param MbottomWheelSpeed If setManually then this will be the bottom wheel Speed
     * @return This command will start the flywheel
     */
    public void updateDistance(double distance, boolean setManually, double MbottomWheelSpeed) {
    
        // if(state == PressureState.HIGH) bottomWheelSpeed = calcWheelSpeedHighPressure(distance);
        // else if(state == PressureState.MID) bottomWheelSpeed = calcWheelSpeedMidPressure(distance);
        // else bottomWheelSpeed = calcWheelSpeedLowPressure(distance);
        double speedsArray[] = clacRpms(distance); 
        
        if(setManually){ update(speedsArray[0], MbottomWheelSpeed); }
        else update(speedsArray[0], speedsArray[1]);
        SmartDashboard.putNumber("ClacTopRPM", speedsArray[0]);
        SmartDashboard.putNumber("ClacBottomRPM", speedsArray[1]);

    }

   

    public double getBottomWheelSpeed() {
        return bottomWheelSpeed;
    }
   

    /**
     * Update the desired state of the flywheels
     * 
     * @param topRPM Angular velocity of top wheel (RPM)
     * @param bottomRPM Angilar velocity of bottom wheel (RPM)
     */
    public void update(double topRPM, double bottomRPM) {
    //    System.out.println("Deired Top" + topRPM);
    //    System.out.println("Desired Bottom" + bottomRPM);
        //Using REVCoder
        desiredSpeedTop = topRPM;
        desiredSpeedBottom = bottomRPM;

        bottomRPM = bottomRPM - 180;

        if(!useInternalEncoders) {
            double topCurPos = topEnc.getDistance();
            double bottomCurPos = bottomEnc.getDistance();
            double currentTime = Timer.getFPGATimestamp() / 60.0; //(System.currentTimeMillis() / 1000.0) / 60.0;

            double topDeltaP = topCurPos - topLastPos;
            double bottomDeltaP = bottomCurPos - bottomLastPos;

            double deltaT = currentTime - lastTime;

            topV = topDeltaP / deltaT;
            bottomV = bottomDeltaP / deltaT;

            double topError = topRPM - topV;
            double bottomError = bottomRPM - bottomV;

            double pidOutput_top = kP_top * topError;
            double pidOutput_bottom = kP_bottom * bottomError;

            double bottomFeedforwardOutput = feedforward_bottom.calculate(bottomRPM / 60);
            double topFeedforwardOutput = feedforward_top.calculate(topRPM / 60);

            double bottomOutput;
            if(Math.abs(pidOutput_bottom) > 0) bottomOutput = (bottomFeedforwardOutput + pidOutput_bottom) / bottomMotor.getBusVoltage();
            else bottomOutput = bottomFeedforwardOutput / bottomMotor.getBusVoltage();

            double topOutput;
            if(Math.abs(pidOutput_top) > 0) topOutput = (topFeedforwardOutput + pidOutput_top) / topMotor.getBusVoltage();
            else topOutput = topFeedforwardOutput / topMotor.getBusVoltage();

            bottomMotor.set(ControlMode.PercentOutput, bottomOutput);
            topMotor.set(ControlMode.PercentOutput, topOutput);

            topLastPos = topEnc.getDistance();
            bottomLastPos = bottomEnc.getDistance();
            lastTime = Timer.getFPGATimestamp() / 60.0; //(System.currentTimeMillis() / 1000.0) / 60.0;
        } else {
            double vTop = rpmToVelUnits(topRPM) * -1;
            double vBottom = rpmToVelUnits(bottomRPM) * -1;

            //Velocity tracking should be closed loop, in counts/100ms
            if(topMotor.getSupplyCurrent() < 50 && bottomMotor.getSupplyCurrent() < 50 && vTop != 0) {
                bottomMotor.set(ControlMode.Velocity, vBottom);
                topMotor.set(ControlMode.Velocity, vTop);
            }
            else {
                bottomMotor.set(ControlMode.PercentOutput, 0);
                topMotor.set(ControlMode.PercentOutput, 0);
            }

        }
    }

  

    /**
     * Updates raw percentage outputs of the shooter
     * 
     * @param topPercent The percentage at which to run the top motor (-1 to 1)
     * @param bottomPercent The percentage at which to run the bottom motor (-1 to 1)
     */
    public void updatePercentage(double topPercent, double bottomPercent) {
        bottomMotor.set(ControlMode.PercentOutput, -bottomPercent);
        topMotor.set(ControlMode.PercentOutput, -topPercent);

        //System.out.println("Top encoder: " + topEnc.get());
        //System.out.println("Bottom encoder: " + bottomEnc.get());
    }

    public double getTopVel() {return topV;}
    public double getBottomVel() {return bottomV;}

    // Everything below is simulation-specific
    private final double simFlywheelkP = 0.001;

    /**
     * Simulates flywheel and returns current speed of simulation
     * 
     * @param ref The desired speed in RPM
     * @return The current simulation speed in RPM
     */
    public double simulate(double ref) {

        //Simple proportional control for sim testing
        double currentSpeed = sim.getAngularVelocityRPM();
        double error = ref - currentSpeed;
        double output = error * simFlywheelkP * RobotController.getBatteryVoltage();

        sim.setInputVoltage(output);

        //20 ms loop time
        sim.update(0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        return sim.getAngularVelocityRPM();
    }

    /**
     * Gets if the flywheel is moving at the desired speed
     * <p>
     * This is primarly meant to check for updates 
     * @return
     */
    public boolean atDesiredSpeed() {
        return (Math.abs(desiredSpeedTop - (desiredSpeedTop * 0.05)) > getTopVel())
            && (Math.abs(desiredSpeedBottom - (desiredSpeedBottom * 0.05)) > getBottomVel());
    }
    public void atSpeed(){
        SmartDashboard.putBoolean("Shotter Status", atDesiredSpeed());
    }

   
}