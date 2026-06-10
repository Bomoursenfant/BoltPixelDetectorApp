namespace BoltPixelDetectorApp;

/// <summary>Material card with title — replaces flat GroupBox styling.</summary>
public sealed class CardPanel : Panel
{
    private string _title = string.Empty;
    private int _titleBottomGap = UiTheme.SpacingSm;

    public string Title
    {
        get => _title;
        set { _title = value; Invalidate(); }
    }

    /// <summary>Extra space between the painted title and docked child content.</summary>
    public int TitleBottomGap
    {
        get => _titleBottomGap;
        set
        {
            _titleBottomGap = Math.Max(0, value);
            ApplyContentPadding();
        }
    }

    public CardPanel()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        BackColor = Color.Transparent;
        ApplyContentPadding();
    }

    private void ApplyContentPadding()
    {
        // Title band (~26px) + gap before children.
        int top = 26 + _titleBottomGap;
        Padding = new Padding(UiTheme.SpacingSm, top, UiTheme.SpacingSm, UiTheme.SpacingSm);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        e.Graphics.Clear(Parent?.BackColor ?? UiTheme.Surface);
        var bounds = new Rectangle(0, 0, Width - 1, Height - 1);
        UiTheme.DrawRoundedFill(e.Graphics, bounds, UiTheme.RadiusLarge, UiTheme.SurfaceContainer);
        UiTheme.DrawRoundedBorder(e.Graphics, bounds, UiTheme.RadiusLarge, UiTheme.OutlineVariant, 1);

        if (!string.IsNullOrWhiteSpace(_title))
        {
            var titleBounds = new Rectangle(UiTheme.SpacingLg, 10, Width - UiTheme.SpacingLg * 2, 24);
            TextRenderer.DrawText(
                e.Graphics,
                _title,
                UiTheme.TitleFont,
                titleBounds,
                UiTheme.OnSurface,
                TextFormatFlags.Left | TextFormatFlags.VerticalCenter);
        }
    }
}
