package frc.robot.components.shooter;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
 * 
 */
public class Flywheel implements RobotComponent {

    final Robot robot;

    TalonFX motor1, motor2;
    Encoder enc;

    private final double rotationsPerPulse = 1.0 / 8192.0; //8192 encoder counts per revolution

    //For Sim
    DCMotor drive;
    EncoderSim encSim;
    FlywheelSim sim;

    public Flywheel(Robot robot) {
        this.robot = robot;
    }

    @Override
    public RobotComponent init() {       
        //Hardware declarations
        motor1 = new TalonFX(FW_FALCON_1);
        motor2 = new TalonFX(FW_FALCON_2);
        enc = new Encoder(FW_ENC_A, FW_ENC_B, false, EncodingType.k1X);


        //For SIM
        drive = DCMotor.getFalcon500(2);
        sim = new FlywheelSim(
            drive,      //Input Gearbox
            1,          //Gearing
            0.0023      //Moment of inertia
            );

        return this;
    }

    @Override
    public void loop() {
        
    }

    @Override
    public void shutdown() {
        //Close motors (find out how to do that for Talons)
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }

    private final double flywheelkP = 0.001;

    /**
     * Simulates flywheel and returns current speed of simulation
     * @param ref The desired speed in RPM
     * @return The current simulation speed in RPM
     */
    public double simulate(double ref) {

        //Simple proportional control for sim testing
        double currentSpeed = sim.getAngularVelocityRPM();
        double error = ref - currentSpeed;
        double output = error * flywheelkP * RobotController.getBatteryVoltage();

        sim.setInputVoltage(output);

        //20 ms loop time
        sim.update(0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        return sim.getAngularVelocityRPM();
    }
}
