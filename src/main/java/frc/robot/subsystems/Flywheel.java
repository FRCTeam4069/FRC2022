package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Shooter flywheel subsystem */
public class Flywheel implements RobotSubsystem {

    public static final int FW_FALCON_1 = 8;
    public static final int FW_FALCON_2 = 9;
    public static final int FW_ENC_TOP_A = 6;
    public static final int FW_ENC_TOP_B = 7;
    public static final int FW_ENC_BOTTOM_A = 4;
    public static final int FW_ENC_BOTTOM_B = 5;

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

    private double kS_top = 0.19816;
    private double kV_top = 0.4825;
    private double kA_top = 1.6034;

    private double kS_bottom = 1.3065;
    private double kV_bottom = 0.46247;
    private double kA_bottom = 3.4751;

    //For Sim
    DCMotor drive;
    EncoderSim encSim;
    FlywheelSim sim;

    /** @param useInternalEncoders true if Falcon built in encoders are being used */
    public Flywheel(boolean useInternalEncoders) {
        this.useInternalEncoders = useInternalEncoders;
        cpr = useInternalEncoders ? 2048 : 8192;

        //PID top    internalEncoder : REVCoder
        kP_top = useInternalEncoders ? 0 : 0;
        kI_top = useInternalEncoders ? 0 : 0;
        kD_top = useInternalEncoders ? 0 : 0;

        //PID bottom   internalEncoder : REVCoder
        kP_bottom = useInternalEncoders ? 0 : 4.1404;
        kI_bottom = useInternalEncoders ? 0 : 0;
        kD_bototom = useInternalEncoders ? 0 : 0;

        feedforward_top = new SimpleMotorFeedforward(kS_top, kV_top, kA_top);
        feedforward_bottom = new SimpleMotorFeedforward(kS_bottom, kV_bottom, kA_bottom);
    }

    @Override
    public void init() {       
        //Hardware declarations
        topMotor = new TalonFX(FW_FALCON_1);
        bottomMotor = new TalonFX(FW_FALCON_2);

        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        
        if(!useInternalEncoders) {
            topEnc = new Encoder(FW_ENC_TOP_A, FW_ENC_TOP_B, false, EncodingType.k1X);
            bottomEnc = new Encoder(FW_ENC_BOTTOM_A, FW_ENC_BOTTOM_B, false, EncodingType.k1X);
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
     * 
     * @param topRPM Angular velocity of top wheel (RPM)
     * @param bottomRPM Angilar velocity of bottom wheel (RPM)
     */
    public void update(double topRPM, double bottomRPM) {
        //Using REVCoder
        if(!useInternalEncoders) {
            double topCurPos = topEnc.getDistance();
            double bottomCurPos = bottomEnc.getDistance();
            double currentTime = Timer.getFPGATimestamp() / 60.0; //(System.currentTimeMillis() / 1000.0) / 60.0;

            double topDeltaP = topCurPos - topLastPos;
            double bottomDeltaP = bottomCurPos - bottomLastPos;

            double deltaT = currentTime - lastTime;

            double topV = topDeltaP / deltaT;
            double bottomV = bottomDeltaP / deltaT;

            double topError = topRPM - topV;
            double bottomError = bottomRPM - bottomV;

            double pidOutput_top = kP_top * topError;
            double pidOutput_bottom = kP_bottom * bottomError;

            double bottomFeedforwardOutput = feedforward_bottom.calculate(bottomRPM / 60);
            double topFeedforwardOutput = feedforward_top.calculate(topRPM / 60);

            double bottomOutput;
            if(pidOutput_bottom > 0) bottomOutput = (bottomFeedforwardOutput + pidOutput_bottom) / bottomMotor.getBusVoltage();
            else bottomOutput = bottomFeedforwardOutput / bottomMotor.getBusVoltage();

            double topOutput;
            if(pidOutput_top > 0) topOutput = (topFeedforwardOutput + pidOutput_top) / topMotor.getBusVoltage();
            else topOutput = topFeedforwardOutput / topMotor.getBusVoltage();

            bottomMotor.set(ControlMode.PercentOutput, bottomOutput);
            topMotor.set(ControlMode.PercentOutput, topOutput);
            
            // System.out.println("Top Vel: " + topV);
            // System.out.println("Top Percent Output: " + topMotor.getMotorOutputPercent());
            // System.out.println("Top Current Draw: " + topMotor.getSupplyCurrent());

            System.out.println("Bottom Vel: " + bottomV);
            // System.out.println("Bottom Percent Output: " + bottomMotor.getMotorOutputPercent());
            // System.out.println("Bottom Current Draw: " + bottomMotor.getSupplyCurrent());

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

            System.out.println("Top Vel: " + (topRPM + velUnitsToRPM(topMotor.getClosedLoopError())));
            System.out.println("Bottom Vel: " + (bottomRPM + velUnitsToRPM(bottomMotor.getClosedLoopError())));

            System.out.println("Top Percent Output: " + topMotor.getMotorOutputPercent());
            System.out.println("Bottom Percent Output: " + bottomMotor.getMotorOutputPercent());

            System.out.println("Top Voltage Draw: " + topMotor.getMotorOutputVoltage());
            System.out.println("Bottom Voltage Draw: " + bottomMotor.getMotorOutputVoltage()); 
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

        System.out.println("Top encoder: " + topEnc.get());
        System.out.println("Bottom encoder: " + bottomEnc.get());
    }

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
}
