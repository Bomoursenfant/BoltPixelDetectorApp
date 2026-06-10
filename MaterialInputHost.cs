using System.Drawing.Drawing2D;

namespace BoltPixelDetectorApp;

/// <summary>White rounded field (opaque paint, clipped corners).</summary>
public sealed class MaterialInputHost : Panel
{
    private const int PaddingTop = 10;
    private const int PaddingBottom = 3;

    public const int DefaultHeight = 34;

    public MaterialInputHost()
    {
        BackColor = UiTheme.SurfaceContainer;
        Padding = new Padding(UiTheme.SpacingSm + 2, PaddingTop, UiTheme.SpacingSm + 2, PaddingBottom);
        Margin = new Padding(0, 1, 0, 2);
        MinimumSize = new Size(60, DefaultHeight);
        Height = DefaultHeight;
        SetStyle(
            ControlStyles.AllPaintingInWmPaint |
            ControlStyles.OptimizedDoubleBuffer |
            ControlStyles.UserPaint |
            ControlStyles.ResizeRedraw,
            true);
    }

    public static MaterialInputHost Wrap(Control inner)
    {
        var host = new MaterialInputHost();
        if (inner is NumericUpDown)
        {
            // Keep spinner away from the rounded right/bottom corners.
            host.Padding = new Padding(UiTheme.SpacingSm + 2, PaddingTop, UiTheme.SpacingSm + 8, PaddingBottom);
        }

        inner.Dock = DockStyle.Fill;
        inner.Margin = Padding.Empty;
        host.Controls.Add(inner);
        UiTheme.StyleVisionField(inner);
        return host;
    }

    protected override void OnResize(EventArgs e)
    {
        base.OnResize(e);
        if (Width > 2 && Height > 2)
            UiTheme.ApplyRoundedRegion(this, UiTheme.RadiusSmall);
    }

    protected override void OnPaintBackground(PaintEventArgs pevent)
    {
        pevent.Graphics.Clear(Parent?.BackColor ?? UiTheme.SurfaceContainer);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        if (ClientRectangle.Width <= 0 || ClientRectangle.Height <= 0)
            return;

        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.PixelOffsetMode = PixelOffsetMode.HighQuality;

        var bounds = new Rectangle(0, 0, Width - 1, Height - 1);
        UiTheme.DrawRoundedFill(g, bounds, UiTheme.RadiusSmall, Color.White);
        UiTheme.DrawRoundedBorder(g, bounds, UiTheme.RadiusSmall, UiTheme.OutlineVariant, 1);
    }
}
