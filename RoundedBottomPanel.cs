namespace BoltPixelDetectorApp;

/// <summary>Panel with rounded bottom-left and bottom-right corners (flat top).</summary>
public sealed class RoundedBottomPanel : Panel
{
    private int _cornerRadius = UiTheme.RadiusLarge;
    private Color _fillColor = UiTheme.Surface;
    private Color _borderColor = UiTheme.OutlineVariant;
    private int _borderWidth = 1;
    private Padding _contentPadding = new(UiTheme.SpacingXs, 0, UiTheme.SpacingXs, UiTheme.SpacingXs);

    public int CornerRadius
    {
        get => _cornerRadius;
        set { _cornerRadius = value; Invalidate(); }
    }

    public Color FillColor
    {
        get => _fillColor;
        set { _fillColor = value; Invalidate(); }
    }

    public Color BorderColor
    {
        get => _borderColor;
        set { _borderColor = value; Invalidate(); }
    }

    public int BorderWidth
    {
        get => _borderWidth;
        set { _borderWidth = value; Invalidate(); }
    }

    public Padding ContentPadding
    {
        get => _contentPadding;
        set { _contentPadding = value; Padding = value; Invalidate(); }
    }

    public RoundedBottomPanel()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        BackColor = Color.Transparent;
        Padding = _contentPadding;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        e.Graphics.Clear(Parent?.BackColor ?? UiTheme.Surface);
        var bounds = new Rectangle(0, 0, Width - 1, Height - 1);
        using var path = UiTheme.CreateBottomRoundedRectanglePath(bounds, _cornerRadius);
        e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        using (var brush = new SolidBrush(_fillColor))
            e.Graphics.FillPath(brush, path);
        if (_borderWidth > 0)
            UiTheme.DrawBottomRoundedBorder(e.Graphics, bounds, _cornerRadius, _borderColor, _borderWidth);
    }
}
