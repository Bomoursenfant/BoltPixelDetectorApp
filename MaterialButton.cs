using System.Drawing.Drawing2D;

namespace BoltPixelDetectorApp;

public enum MaterialButtonVariant
{
    Filled,
    FilledTonal,
    Outlined,
    Text
}

/// <summary>Material Design 3–style button with rounded corners.</summary>
public sealed class MaterialButton : Button
{
    private MaterialButtonVariant _variant = MaterialButtonVariant.Filled;
    private bool _hover;
    private bool _pressed;

    public MaterialButtonVariant Variant
    {
        get => _variant;
        set { _variant = value; Invalidate(); }
    }

    public MaterialButton()
    {
        FlatStyle = FlatStyle.Flat;
        FlatAppearance.BorderSize = 0;
        UseVisualStyleBackColor = false;
        Cursor = Cursors.Hand;
        Font = UiTheme.ButtonFont;
        Height = 42;
        MinimumSize = new Size(96, 42);
        Padding = new Padding(UiTheme.SpacingMd, UiTheme.SpacingSm, UiTheme.SpacingMd, UiTheme.SpacingSm);
        AutoSize = false;
        BackColor = Color.Transparent;
        SetStyle(
            ControlStyles.AllPaintingInWmPaint |
            ControlStyles.OptimizedDoubleBuffer |
            ControlStyles.UserPaint |
            ControlStyles.ResizeRedraw |
            ControlStyles.SupportsTransparentBackColor,
            true);
    }

    protected override void OnPaintBackground(PaintEventArgs pevent)
    {
        // Prevent default WinForms button background (black corners on custom paint).
    }

    protected override void OnMouseEnter(EventArgs e)
    {
        base.OnMouseEnter(e);
        _hover = true;
        Invalidate();
    }

    protected override void OnMouseLeave(EventArgs e)
    {
        base.OnMouseLeave(e);
        _hover = false;
        _pressed = false;
        Invalidate();
    }

    protected override void OnMouseDown(MouseEventArgs mevent)
    {
        base.OnMouseDown(mevent);
        if (mevent.Button == MouseButtons.Left)
        {
            _pressed = true;
            Invalidate();
        }
    }

    protected override void OnMouseUp(MouseEventArgs mevent)
    {
        base.OnMouseUp(mevent);
        _pressed = false;
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs pevent)
    {
        var g = pevent.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.PixelOffsetMode = PixelOffsetMode.HighQuality;

        Color parentBack = GetParentBackground();
        using (var clear = new SolidBrush(parentBack))
            g.FillRectangle(clear, ClientRectangle);

        if (ClientRectangle.Width <= 0 || ClientRectangle.Height <= 0)
            return;

        var bounds = ClientRectangle;
        GetColors(out Color back, out Color fore, out Color border);

        if (_pressed)
            back = Blend(back, UiTheme.OnSurface, 0.12f);
        else if (_hover)
            back = Blend(back, _variant is MaterialButtonVariant.Filled or MaterialButtonVariant.FilledTonal
                ? Color.White
                : UiTheme.Primary, 0.08f);

        int radius = Math.Min(UiTheme.RadiusMedium, Math.Min(bounds.Width, bounds.Height) / 2);
        UiTheme.DrawRoundedFill(g, bounds, radius, back);
        if (_variant == MaterialButtonVariant.Outlined && border != Color.Transparent)
            UiTheme.DrawRoundedBorder(g, bounds, radius, border, 1);

        TextRenderer.DrawText(
            g,
            Text,
            Font,
            bounds,
            fore,
            TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis);
    }

    private Color GetParentBackground()
    {
        for (Control? parent = Parent; parent is not null; parent = parent.Parent)
        {
            if (parent.BackColor != Color.Transparent)
                return parent.BackColor;
        }

        return UiTheme.SurfaceContainer;
    }

    private void GetColors(out Color back, out Color fore, out Color border)
    {
        border = Color.Transparent;
        switch (_variant)
        {
            case MaterialButtonVariant.Filled:
                back = UiTheme.Primary;
                fore = UiTheme.OnPrimary;
                break;
            case MaterialButtonVariant.FilledTonal:
                back = UiTheme.SecondaryContainer;
                fore = UiTheme.OnPrimaryContainer;
                break;
            case MaterialButtonVariant.Outlined:
                back = UiTheme.SurfaceContainerHighest;
                fore = UiTheme.Primary;
                border = UiTheme.Outline;
                break;
            default:
                back = Color.Transparent;
                fore = UiTheme.Primary;
                break;
        }
    }

    private static Color Blend(Color a, Color b, float amount)
    {
        amount = Math.Clamp(amount, 0f, 1f);
        int r = (int)(a.R + (b.R - a.R) * amount);
        int g = (int)(a.G + (b.G - a.G) * amount);
        int bl = (int)(a.B + (b.B - a.B) * amount);
        return Color.FromArgb(a.A, r, g, bl);
    }
}
