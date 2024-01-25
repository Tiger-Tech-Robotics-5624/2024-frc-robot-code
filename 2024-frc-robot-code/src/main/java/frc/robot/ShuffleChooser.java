package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleChooser {
    private SendableChooser<String> start;
    private SendableChooser<Boolean> balance;
    
    public ShuffleChooser() {
        start = new SendableChooser<>();
        balance = new SendableChooser<>();
        configStartScore();
        configureLocation();


    } 

    public void configStartScore() {
        start.setDefaultOption("Cube", "CUBE");
        start.addOption("Cube","CUBE");
        start.addOption("Cone", "CONE");
    }

    public void configureLocation() {
        balance.setDefaultOption("Sides", false);
        balance.addOption("Mid",true);
        balance.addOption("Sides", false);
    }

    public SendableChooser<String> getStartItem() {
        return this.start;
    }

    public SendableChooser<Boolean> getStartLocation() {
        return this.balance;
    }
}
