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
        float totalRolling = 0;
        float totalSkidding = 0;
        float totalRubbing = 0;

        foreach (var wheel in wheels)
        {
            totalRolling += Mathf.Abs(wheel.spin);
            totalSkidding += Mathf.Abs(wheel.outputSkidding);
            totalRubbing += Mathf.Abs(wheel.outputStressing);
        }

        totalSkidding = Mathf.Max(totalSkidding - 20f, 0f) * 0.25f;

        totalRubbing = Mathf.Max(totalRubbing - 10, 0);

        rolling.PitchScale = 1.0f + totalRolling / 1000;
        rolling.VolumeDb = Mathf.Clamp(Mathf.LinearToDb(Mathf.Max(totalRolling / 1000.0f, 0.0f) * 0.75f), -80.0f, 0.0f);
        rolling.MaxDb = rolling.VolumeDb;

        rub.VolumeDb = Mathf.Clamp(Mathf.LinearToDb(Mathf.Max(totalRubbing - totalSkidding, 0.0f) * 0.75f / 300.0f), -80.0f, 0.0f);
        rub.MaxDb = rub.VolumeDb;

        skid.VolumeDb = Mathf.Clamp(Mathf.LinearToDb(Mathf.Max(totalSkidding, 0.0f) / 30.0f * 0.75f), -80.0f, 0.0f);
        skid.MaxDb = skid.VolumeDb;
    }
}
