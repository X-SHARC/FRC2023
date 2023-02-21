package frc.robot.lib.vision;

public class VisionTarget {
    private double yaw;
    private double pitch;
    private double targetHeight;
    private double cameraHeight;
    private double cameraAngle;

    public VisionTarget(double targetHeight, double cameraHeight, double cameraAngle){
        this.targetHeight = targetHeight;
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
    }
    
    public void updateYaw(double yaw){
        this.yaw = yaw;
    }

    public void updatePitch(double pitch){
        this.pitch = pitch;
    }

    public void update(double yaw, double pitch){
        this.yaw = yaw;
        this.pitch = pitch;
    }

    public double getYaw(){
        return yaw;
    }

    public double getPitch(){
        return pitch;
    }

    public double getDistance(){
        double distance = (targetHeight-cameraHeight) / Math.tan(Math.toRadians(pitch + cameraAngle));
        return distance;
    }


}
