package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;


public class TurretLimeTurn extends CommandBase{

    private final Turret m_turret;
    final double STEER_K = 0.15;
    private boolean stop = false;
    private double x;
    private double v;
    private double adjustment = 0.0;
    private double prev_tx = 1.0;
    private double steering_factor = 0.25;
    public boolean stopSteer = false;
    private boolean newPIDLoop = false;
    private double Kp = -0.1;
    private double min_command = 0.05;
    public PIDController turnController;
  

    public TurretLimeTurn(Turret subsystem){

        m_turret = subsystem;
        addRequirements(m_turret);
    }

    public void initialize(){

         turnController = new PIDController(0.01, 0.01, 0);


        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        // pidController.setPID(0.015, 0, 0);
        // pidController.setTolerance(0.01);
    }

    public void execute(){

        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double heading_error = -x;
        double steering_adjust = 0.0;

        // if (v == 1 && !stopSteer){

        //     adjustment = pidController.calculate(x);
        //     prev_tx = x;

        //     if (!newPIDLoop) {
        //         newPIDLoop = true;
        //         pidController.setSetpoint(Math.signum(prev_tx) );
        //       }


        // }else{

        //     newPIDLoop = false;
        //     pidController.reset();
        //     adjustment = Math.signum(prev_tx) * steering_factor;

        // }

        // if (Math.abs(x) < 1.0 && Math.abs(prev_tx) < 1.0) stopSteer = true;
        // else stopSteer = false;
        // if(stopSteer) {
        //   adjustment = 0;
        // }

        // adjustment = Math.signum(x) * Math.min(Math.abs(adjustment), 0.5);

        // m_turret.turn(adjustment);
        //SmartDashboard.putNumber("Adjustment", adjustment);
      

        // left_command += steering_adjust;
        // right_command -= steering_adjust;
        if (v == 1.0) {
         
            m_turret.turn(-turnController.calculate(x, 0));
        
        }else {

            stop = true;
           // System.out.println("Not in Sight");

            // while(Math.abs(x) > 10){

            // x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            // v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
            //     m_turret.turn(0.3 * Math.signum(x));

            //     System.out.println("faster");
                

            // }
            // while(Math.abs(x) > 2 && Math.abs(x) < 10){

            //     x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            //     v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
            //     m_turret.turn(0.1 * Math.signum(x));
            //     System.out.println("slower");
               
            // }
           // stop = true;
            }
        }

    // //    stop = true;
    // }
    public boolean isFinished(){
       // System.out.println("finished");
        return stop;
     }

}