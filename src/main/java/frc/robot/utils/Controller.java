package frc.robot.utils;

public class Controller {
    public int id;
    public StickDeadband leftStick;
    public StickDeadband rightStick;

    public Controller(int id, StickDeadband leftStick, StickDeadband rightStick) {
        this.id = id;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
    }
}