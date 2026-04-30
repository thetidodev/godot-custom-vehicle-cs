using Godot;
using Godot.Collections;


public partial class Debug : Control
{
    [Export] VBoxContainer list;

    Dictionary<string, Label> debugLabels = [];

    public override void _PhysicsProcess(double delta)
    {
        AddOrUpdateDebug("FPS", Engine.GetFramesPerSecond().ToString());
    }

    public void AddOrUpdateDebug(string name, string value)
    {
        if (debugLabels.TryGetValue(name, out Label label))
        {
            label.Text = name + " : " + value;
        }
        else
        {
            label = new Label()
            {
                Text = name + " : " + value
            };
            debugLabels.Add(name, label);
            list.AddChild(label);
        }
    }
}
