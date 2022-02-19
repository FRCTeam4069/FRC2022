package frc.robot.components.shooter;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;
import frc.robot.components.RobotComponent;

import static frc.robot.Constants.*;
/**
 * Shooter flywheel control
 */
public class Flywheel implements RobotComponent {

    final Robot robot;

    TalonFX topMotor, bottomMotor;
    Encoder topEnc, bottomEnc;

    private int cpr;
    private double kP_top;
    private double kI_top;
    private double kD_top;

    private double kP_bottom;
    private double kI_bottom;
    private double kD_bototom;

    boolean useInternalEncoders = false;

    SimpleMotorFeedforward feedforward_top;
    SimpleMotorFeedforward feedforward_bottom;

    private double kS_top = 0;
    private double kV_top = 0;
    private double kA_top = 0;

    private double kS_bottom = 0;
    private double kV_bottom = 0;
    private double kA_bottom = 0;

    //For Sim
    DCMotor drive;
    EncoderSim encSim;
    FlywheelSim sim;

    /**
     * @param robot robot instance
     * @param useInternalEncoders true if Falcon built in encoders are being used
     */
    public Flywheel(Robot robot, boolean useInternalEncoders) {
        this.robot = robot;
        this.useInternalEncoders = useInternalEncoders;
        cpr = useInternalEncoders ? 2048 : 8192;

        //PID top    internalEncoder : REVCoder
        kP_top = useInternalEncoders ? 0 : 0;
        kI_top = useInternalEncoders ? 0 : 0;
        kD_top = useInternalEncoders ? 0 : 0;


        //PID bottom   internalEncoder : REVCoder
        kP_bottom = useInternalEncoders ? 0 : 0;
        kI_bottom = useInternalEncoders ? 0 : 0;
        kD_bototom = useInternalEncoders ? 0 : 0;

        feedforward_top = new SimpleMotorFeedforward(kS_top, kV_top, kA_top);
        feedforward_bottom = new SimpleMotorFeedforward(kS_bottom, kV_bottom, kA_bottom);
    }

    @Override
    public RobotComponent init() {       
        //Hardware declarations
        topMotor = new TalonFX(FW_FALCON_1);
        bottomMotor = new TalonFX(FW_FALCON_2);
        
        if(!useInternalEncoders) {
            topEnc = new Encoder(FW_ENC_TOP_A, FW_ENC_TOP_B, false, EncodingType.k1X);
            bottomEnc = new Encoder(FW_ENC_BOTTOM_A, FW_ENC_BOTTOM_B, false, EncodingType.k1X);
            topEnc.setDistancePerPulse(1 / 8192);
            bottomEnc.setDistancePerPulse(1 / 8192);
        }


        //For SIM
        if(!robot.isReal()) {
            drive = DCMotor.getFalcon500(2);
            sim = new FlywheelSim(
                drive,      //Input Gearbox
                1,          //Gearing
                0.0023      //Moment of inertia
                );
        }
        return this;
    }

    @Override
    public void loop() {
        
    }

    //For Update
    double topLastPos = 0;
    double bottomLastPos = 0;
    double lastTime = System.currentTimeMillis();

    //Conversions between RPM and TalonFX velocity units
    private double rpmToVelUnits(double rpm) {
        return (rpm / 600) * cpr;
    }

    private double velUnitsToRPM(double velUnits) {
        return (velUnits / cpr) * 600;
    }

    /**
     * Update the desired state of the flywheels
     * @param topRPM Angular velocity of top wheel (RPM)
     * @param bottomRPM Angilar velocity of bottom wheel (RPM)
     */
    public void update(double topRPM, double bottomRPM) {
        
        //Using REVCoder
        if(!useInternalEncoders) {
            double topCurPos = topEnc.getDistance();
            double bottomCurPos = bottomEnc.getDistance();
            double currentTime = System.currentTimeMillis() * 1000 * 60;

            double topDeltaP = topCurPos - topLastPos;
            double bottomDeltaP = bottomCurPos - bottomLastPos;

            double deltaT = currentTime - lastTime;

            double topV = topDeltaP / deltaT;
            double bottomV = bottomDeltaP / deltaT;

            double topError = topRPM - topV;
            double bottomError = bottomRPM - bottomV;

            double pidOutput_top = kP_top * topError;
            double pidOutput_bottom = kP_bottom * bottomError;

            double bottomFeedforwardOutput = feedforward_bottom.calculate(bottomRPM);
            double topFeedforwardOutput = feedforward_top.calculate(topRPM);

            double bottomOutput = (bottomFeedforwardOutput + pidOutput_bottom) / 12.0;
            double topOutput = (topFeedforwardOutput + pidOutput_top) / 12.0;

            //Insert stuff for integral and derivative control if you go that route
            if(topMotor.getSupplyCurrent() < 50 && bottomMotor.getSupplyCurrent() < 50) {
                if(topError > 0) topMotor.set(ControlMode.PercentOutput, topOutput);
                else topMotor.set(ControlMode.PercentOutput, 0);
                if(bottomError > 0) bottomMotor.set(ControlMode.PercentOutput, bottomOutput);
                else bottomMotor.set(ControlMode.PercentOutput, 0);
            }
            
            System.out.println("Top Vel: " + topV);
            System.out.println("Top Percent Output: " + topMotor.getMotorOutputPercent());
            System.out.println("Top Current Draw: " + topMotor.getSupplyCurrent());

            System.out.println("Bottom Vel: " + bottomV);
            System.out.println("Bottom Percent Output: " + bottomMotor.getMotorOutputPercent());
            System.out.println("Bottom Current Draw: " + bottomMotor.getSupplyCurrent());

            topLastPos = topEnc.getDistance();
            bottomLastPos = bottomEnc.getDistance();
            lastTime = System.currentTimeMillis();
        }
        else {

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

            System.out.println("Top Vel: " + (topRPM + velUnitsToRPM(topMotor.getClosedLoopError())));
            System.out.println("Bottom Vel: " + (bottomRPM + velUnitsToRPM(bottomMotor.getClosedLoopError())));

            System.out.println("Top Percent Output: " + topMotor.getMotorOutputPercent());
            System.out.println("Bottom Percent Output: " + bottomMotor.getMotorOutputPercent());

            System.out.println("Top Voltage Draw: " + topMotor.getMotorOutputVoltage());
            System.out.println("Bottom Voltage Draw: " + bottomMotor.getMotorOutputVoltage()); 
        }
    }

    // Everything below is simulation-specific
    private final double simFlywheelkP = 0.001;

    /**
     * Simulates flywheel and returns current speed of simulation
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
}
