namespace BoltPixelDetectorApp;

/// <summary>Rounded container for camera / mask previews (Material surface).</summary>
public sealed class RoundedPanel : Panel
{
    private int _cornerRadius = UiTheme.RadiusLarge;
    private Color _borderColor = UiTheme.OutlineVariant;
    private Color _fillColor = UiTheme.ViewportBackground;
    private int _borderWidth = 1;
    private int _innerPadding = UiTheme.SpacingSm;

    public int CornerRadius
    {
        get => _cornerRadius;
        set { _cornerRadius = value; Invalidate(); }
    }

    public Color BorderColor
    {
        get => _borderColor;
        set { _borderColor = value; Invalidate(); }
    }

    public Color FillColor
    {
        get => _fillColor;
        set { _fillColor = value; Invalidate(); }
    }

    public int BorderWidth
    {
        get => _borderWidth;
        set { _borderWidth = value; Invalidate(); }
    }

    public int InnerPadding
    {
        get => _innerPadding;
        set { _innerPadding = value; PerformLayout(); Invalidate(); }
    }

    public RoundedPanel()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        BackColor = Color.Transparent;
        Padding = new Padding(UiTheme.SpacingSm);
    }

    protected override void OnResize(EventArgs eventargs)
    {
        base.OnResize(eventargs);
        UiTheme.ApplyRoundedRegion(this, _cornerRadius);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        e.Graphics.Clear(Parent?.BackColor ?? UiTheme.Surface);
        var bounds = new Rectangle(0, 0, Width - 1, Height - 1);
        UiTheme.DrawRoundedFill(e.Graphics, bounds, _cornerRadius, _fillColor);
        if (_borderWidth > 0)
            UiTheme.DrawRoundedBorder(e.Graphics, bounds, _cornerRadius, _borderColor, _borderWidth);
    }

    protected override void OnControlAdded(ControlEventArgs e)
    {
        base.OnControlAdded(e);
        if (e.Control is PictureBox picture)
        {
            picture.BackColor = FillColor;
            UiTheme.ApplyRoundedRegion(picture, Math.Max(4, _cornerRadius - _innerPadding));
            picture.Resize += (_, _) => UiTheme.ApplyRoundedRegion(picture, Math.Max(4, _cornerRadius - _innerPadding));
        }
    }
}
