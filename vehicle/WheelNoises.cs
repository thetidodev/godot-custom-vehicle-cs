using Godot;
using Godot.Collections;


public partial class WheelNoises : Node3D
{
    [Export] Array<Wheel> wheels;
    [Export] AudioStreamPlayer3D rolling;
    [Export] AudioStreamPlayer3D rub;
    [Export] AudioStreamPlayer3D skid;

    public override void _PhysicsProcess(double delta)
    {
        float total_rolling = 0;
        float total_skidding = 0;
        float total_rubbing = 0;

        foreach (var wheel in wheels)
        {
            total_rolling += Mathf.Abs(wheel.spin);
            total_skidding += Mathf.Abs(wheel.outputSkidding);
            total_rubbing += Mathf.Abs(wheel.outputStressing);
        }

        total_skidding = Mathf.Max(total_skidding - 20f, 0f) * 0.25f;

        total_rubbing = Mathf.Max(total_rubbing - 10, 0);

        rolling.PitchScale = 1.0f + total_rolling / 1000;
        rolling.VolumeDb = Mathf.Clamp(Mathf.LinearToDb(Mathf.Max(total_rolling / 1000.0f, 0.0f) * 0.75f), -80.0f, 0.0f);
        rolling.MaxDb = rolling.VolumeDb;

        rub.VolumeDb = Mathf.Clamp(Mathf.LinearToDb(Mathf.Max(total_rubbing - total_skidding, 0.0f) * 0.75f / 300.0f), -80.0f, 0.0f);
        rub.MaxDb = rub.VolumeDb;

        skid.VolumeDb = Mathf.Clamp(Mathf.LinearToDb(Mathf.Max(total_skidding, 0.0f) / 30.0f * 0.75f), -80.0f, 0.0f);
        skid.MaxDb = skid.VolumeDb;
    }
}
