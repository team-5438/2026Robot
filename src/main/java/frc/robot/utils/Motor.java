package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Motor {
    public enum MotorBrand { Spark, Talon };
    public MotorBrand type;
    public int id;

    public TalonFX talon;
    public SparkMax spark;

    public Motor(int id, MotorBrand type) {
        this.id = id;
        this.type = type;

        if (type == MotorBrand.Spark) {
            spark = new SparkMax(id, MotorType.kBrushless);
        } else if (type == MotorBrand.Talon) {
            talon = new TalonFX(id);
        }
    }

    public void set(double percent) {
        if (type == MotorBrand.Spark) {
            spark.set(percent);
        } else if (type == MotorBrand.Talon) {
            talon.set(percent);
        }
    }
}