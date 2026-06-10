using System.Drawing.Drawing2D;

namespace BoltPixelDetectorApp;

/// <summary>Square toggle with rounded corners and tick only (no wrapper field).</summary>
public sealed class SquareToggle : Control
{
    private const int CornerRadius = 5;
    private bool _checked;

    public bool Checked
    {
        get => _checked;
        set
        {
            if (_checked == value)
                return;
            _checked = value;
            Invalidate();
            CheckedChanged?.Invoke(this, EventArgs.Empty);
        }
    }

    public event EventHandler? CheckedChanged;

    public SquareToggle()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.UserPaint, true);
        Size = new Size(22, 22);
        MinimumSize = new Size(22, 22);
        MaximumSize = new Size(22, 22);
        Cursor = Cursors.Hand;
        TabStop = true;
        BackColor = UiTheme.SurfaceContainer;
    }

    protected override void OnPaintBackground(PaintEventArgs pevent)
    {
        using var brush = new SolidBrush(Parent?.BackColor ?? UiTheme.SurfaceContainer);
        pevent.Graphics.FillRectangle(brush, ClientRectangle);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        var box = new Rectangle(1, 1, Width - 3, Height - 3);
        UiTheme.DrawRoundedFill(g, box, CornerRadius, _checked ? UiTheme.PrimaryContainer : Color.White);
        UiTheme.DrawRoundedBorder(g, box, CornerRadius, _checked ? UiTheme.Primary : UiTheme.Outline, 1);

        if (_checked)
        {
            float cx = box.Left + box.Width / 2f;
            float cy = box.Top + box.Height / 2f;
            const float tickScale = 0.62f;
            var points = new[]
            {
                ScaleTickPoint(box.Left + 4, box.Top + box.Height / 2, cx, cy, tickScale),
                ScaleTickPoint(box.Left + box.Width / 2 - 1, box.Bottom - 4, cx, cy, tickScale),
                ScaleTickPoint(box.Right - 3, box.Top + 4, cx, cy, tickScale)
            };
            using var tickPen = new Pen(UiTheme.Primary, 1.25f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            g.DrawLines(tickPen, points);
        }
    }

    private static Point ScaleTickPoint(int x, int y, float centerX, float centerY, float scale) =>
        new(
            (int)(centerX + (x - centerX) * scale),
            (int)(centerY + (y - centerY) * scale));

    protected override void OnClick(EventArgs e)
    {
        base.OnClick(e);
        Checked = !Checked;
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (e.KeyCode is Keys.Space or Keys.Enter)
        {
            Checked = !Checked;
            e.Handled = true;
        }
    }
}
