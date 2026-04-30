using Godot;
using System;

public partial class CarMesh : Node3D
{
    [Export] bool isGoofEnabled = false;
    [Export] Curve horizontalCurve;
    [Export] Curve verticalCurve;
    double timer = 0;
    Vector3 fullVector = new(1, 1, 1);


    public override void _PhysicsProcess(double delta)
    {
        if (!isGoofEnabled)
        {
            Scale = new(1, 1, 1);
            return;
        }
        Vector3 hehe = new(horizontalCurve.SampleBaked((float)(timer % 1)), verticalCurve.SampleBaked((float)(timer % 1)), horizontalCurve.SampleBaked((float)(timer % 1)));
        Scale = hehe;
        timer += delta * 2;
    }
}
