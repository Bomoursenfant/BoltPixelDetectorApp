using System.Drawing.Drawing2D;

namespace BoltPixelDetectorApp;

/// <summary>Material Design 3–inspired color tokens and shared WinForms styling.</summary>
public static class UiTheme
{
    // Surface & layout
    public static readonly Color Surface = Color.FromArgb(255, 251, 254);
    public static readonly Color SurfaceDim = Color.FromArgb(244, 239, 244);
    public static readonly Color SurfaceContainer = Color.FromArgb(243, 237, 247);
    public static readonly Color SurfaceContainerHigh = Color.FromArgb(236, 230, 240);
    public static readonly Color SurfaceContainerHighest = Color.FromArgb(230, 224, 233);

    // Brand (Material primary palette)
    public static readonly Color Primary = Color.FromArgb(103, 80, 164);
    public static readonly Color OnPrimary = Color.White;
    public static readonly Color PrimaryContainer = Color.FromArgb(234, 221, 255);
    public static readonly Color OnPrimaryContainer = Color.FromArgb(33, 0, 94);

    // Secondary / accents
    public static readonly Color Secondary = Color.FromArgb(98, 91, 113);
    public static readonly Color SecondaryContainer = Color.FromArgb(232, 222, 248);
    public static readonly Color Tertiary = Color.FromArgb(125, 82, 96);
    public static readonly Color TertiaryContainer = Color.FromArgb(255, 216, 228);

    // Content
    public static readonly Color OnSurface = Color.FromArgb(28, 27, 31);
    public static readonly Color OnSurfaceVariant = Color.FromArgb(73, 69, 79);
    public static readonly Color Outline = Color.FromArgb(202, 196, 208);
    public static readonly Color OutlineVariant = Color.FromArgb(222, 216, 225);

    // Semantic
    public static readonly Color Error = Color.FromArgb(186, 26, 26);
    public static readonly Color ErrorContainer = Color.FromArgb(255, 218, 214);
    public static readonly Color Success = Color.FromArgb(46, 125, 50);
    public static readonly Color SuccessContainer = Color.FromArgb(200, 230, 201);
    public static readonly Color Warning = Color.FromArgb(237, 108, 2);
    public static readonly Color WarningContainer = Color.FromArgb(255, 224, 178);

    public static readonly Color ViewportBackground = Color.FromArgb(30, 28, 34);

    public const int RadiusSmall = 8;
    public const int RadiusMedium = 12;
    public const int RadiusLarge = 16;
    public const int RadiusXLarge = 20;

    public const int SpacingXs = 4;
    public const int SpacingSm = 8;
    public const int SpacingMd = 12;
    public const int SpacingLg = 16;
    public const int SpacingXl = 20;

    public static Font UiFont { get; private set; } = new("Segoe UI", 9.25f);
    public static Font UiFontSmall { get; private set; } = new("Segoe UI", 8.75f);
    public static Font TitleFont { get; private set; } = new("Segoe UI Semibold", 10.5f);
    public static Font HeadlineFont { get; private set; } = new("Segoe UI Semibold", 13f);
    public static Font ButtonFont { get; private set; } = new("Segoe UI Semibold", 9.25f);

    public static void Initialize()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
    }

    public static void ApplyToForm(Form form)
    {
        form.BackColor = Surface;
        form.Font = UiFont;
        form.ForeColor = OnSurface;
        form.Padding = Padding.Empty;
    }

    public static void StyleVisionField(Control inner)
    {
        switch (inner)
        {
            case TextBox textBox:
                textBox.BorderStyle = BorderStyle.None;
                textBox.BackColor = Color.White;
                textBox.ForeColor = OnSurface;
                textBox.Font = UiFontSmall;
                textBox.TextAlign = HorizontalAlignment.Left;
                break;
            case ComboBox combo:
                combo.FlatStyle = FlatStyle.Flat;
                combo.BackColor = Color.White;
                combo.ForeColor = OnSurface;
                combo.Font = UiFontSmall;
                break;
            case NumericUpDown numeric:
                numeric.BorderStyle = BorderStyle.None;
                numeric.BackColor = Color.White;
                numeric.ForeColor = OnSurface;
                numeric.Font = UiFontSmall;
                numeric.TextAlign = HorizontalAlignment.Left;
                break;
        }
    }

    public static void StyleLabel(Label label, bool secondary = false, bool title = false)
    {
        label.ForeColor = secondary ? OnSurfaceVariant : OnSurface;
        label.Font = title ? TitleFont : UiFont;
        label.BackColor = Color.Transparent;
    }

    public static void StyleTextBox(TextBox textBox)
    {
        textBox.BorderStyle = BorderStyle.FixedSingle;
        textBox.BackColor = SurfaceContainerHighest;
        textBox.ForeColor = OnSurface;
        textBox.Font = UiFont;
    }

    public static void StyleNumeric(NumericUpDown numeric)
    {
        numeric.BackColor = SurfaceContainerHighest;
        numeric.ForeColor = OnSurface;
        numeric.Font = UiFont;
        numeric.BorderStyle = BorderStyle.FixedSingle;
    }

    public static void StyleComboBox(ComboBox combo)
    {
        combo.FlatStyle = FlatStyle.Flat;
        combo.BackColor = SurfaceContainerHighest;
        combo.ForeColor = OnSurface;
        combo.Font = UiFont;
    }

    public static void StyleCheckBox(CheckBox checkBox)
    {
        checkBox.ForeColor = OnSurface;
        checkBox.Font = UiFont;
        checkBox.FlatStyle = FlatStyle.Flat;
        checkBox.BackColor = Color.Transparent;
    }

    public static void StyleDataGridView(DataGridView grid)
    {
        grid.BackgroundColor = Surface;
        grid.GridColor = OutlineVariant;
        grid.BorderStyle = BorderStyle.None;
        grid.CellBorderStyle = DataGridViewCellBorderStyle.SingleHorizontal;
        grid.RowHeadersVisible = false;
        grid.EnableHeadersVisualStyles = false;
        grid.SelectionMode = DataGridViewSelectionMode.FullRowSelect;
        grid.DefaultCellStyle.BackColor = Surface;
        grid.DefaultCellStyle.ForeColor = OnSurface;
        grid.DefaultCellStyle.SelectionBackColor = PrimaryContainer;
        grid.DefaultCellStyle.SelectionForeColor = OnPrimaryContainer;
        grid.DefaultCellStyle.Font = UiFont;
        grid.DefaultCellStyle.Padding = new Padding(4, 2, 4, 2);
        grid.AlternatingRowsDefaultCellStyle.BackColor = SurfaceDim;
        grid.ColumnHeadersDefaultCellStyle.BackColor = SurfaceContainerHigh;
        grid.ColumnHeadersDefaultCellStyle.ForeColor = OnSurface;
        grid.ColumnHeadersDefaultCellStyle.Font = ButtonFont;
        grid.ColumnHeadersDefaultCellStyle.Padding = new Padding(8, 6, 8, 6);
        grid.ColumnHeadersHeight = 40;
        grid.RowTemplate.Height = 32;
    }

    /// <summary>Wide columns, wrapped headers, default WinForms scrollbars.</summary>
    public static void StyleScrollableDataGridView(DataGridView grid)
    {
        StyleDataGridView(grid);
        grid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.None;
        grid.ScrollBars = ScrollBars.Both;
        grid.AllowUserToResizeColumns = true;
        grid.AllowUserToOrderColumns = false;
        grid.ColumnHeadersDefaultCellStyle.Font = UiFontSmall;
        grid.ColumnHeadersDefaultCellStyle.WrapMode = DataGridViewTriState.True;
        grid.ColumnHeadersDefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleCenter;
        grid.ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.AutoSize;
        grid.ColumnHeadersHeight = 32;
        grid.DefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleCenter;
    }

    public static int MeasureHeaderMinWidth(string header, Font? font = null, int padding = 18)
    {
        font ??= UiFontSmall;
        var size = TextRenderer.MeasureText(
            header,
            font,
            new Size(int.MaxValue / 4, int.MaxValue),
            TextFormatFlags.NoPadding | TextFormatFlags.SingleLine);
        return Math.Max(48, size.Width + padding);
    }

    public static void StyleStatusBar(Control status)
    {
        status.BackColor = SurfaceContainerHigh;
        status.ForeColor = OnSurfaceVariant;
        status.Font = UiFontSmall;
        status.Padding = new Padding(SpacingMd, 0, SpacingMd, 0);
    }

    public static void StyleIntegrationStatus(Label label)
    {
        label.BackColor = SurfaceContainer;
        label.ForeColor = OnSurfaceVariant;
        label.Font = UiFontSmall;
        label.Padding = new Padding(SpacingMd, SpacingSm, SpacingMd, SpacingSm);
    }

    public static void ApplyToControlTree(Control root)
    {
        foreach (Control control in root.Controls)
        {
            switch (control)
            {
                case MaterialButton:
                case RoundedPanel:
                case CardPanel:
                    break;
                case Button button:
                    StyleStandardButton(button);
                    break;
                case Label label:
                    StyleLabel(label, label.ForeColor == Color.DimGray || label.ForeColor == SystemColors.GrayText);
                    break;
                case TextBox textBox:
                    StyleTextBox(textBox);
                    break;
                case NumericUpDown numeric:
                    StyleNumeric(numeric);
                    break;
                case ComboBox combo:
                    StyleComboBox(combo);
                    break;
                case CheckBox checkBox:
                    StyleCheckBox(checkBox);
                    break;
                case DataGridView grid:
                    StyleDataGridView(grid);
                    break;
                case GroupBox groupBox:
                    groupBox.ForeColor = OnSurface;
                    groupBox.Font = TitleFont;
                    groupBox.BackColor = Surface;
                    break;
                case TableLayoutPanel or FlowLayoutPanel or Panel:
                    if (control.BackColor == SystemColors.Control || control.BackColor == Color.Transparent)
                        control.BackColor = Surface;
                    break;
            }

            if (control.HasChildren)
                ApplyToControlTree(control);
        }
    }

    public static void StyleStandardButton(Button button)
    {
        button.FlatStyle = FlatStyle.Flat;
        button.FlatAppearance.BorderSize = 1;
        button.FlatAppearance.BorderColor = Outline;
        button.BackColor = SurfaceContainerHighest;
        button.ForeColor = OnSurface;
        button.Font = ButtonFont;
        button.Cursor = Cursors.Hand;
        button.Padding = new Padding(SpacingMd, SpacingSm, SpacingMd, SpacingSm);
        button.Height = Math.Max(button.Height, 36);
        ApplyRoundedRegion(button, RadiusMedium);
    }

    public static GraphicsPath CreateRoundedRectanglePath(Rectangle bounds, int radius)
    {
        int r = Math.Max(0, Math.Min(radius, Math.Min(bounds.Width, bounds.Height) / 2));
        var path = new GraphicsPath();
        if (r <= 0)
        {
            path.AddRectangle(bounds);
            return path;
        }

        int d = r * 2;
        path.AddArc(bounds.Left, bounds.Top, d, d, 180, 90);
        path.AddArc(bounds.Right - d, bounds.Top, d, d, 270, 90);
        path.AddArc(bounds.Right - d, bounds.Bottom - d, d, d, 0, 90);
        path.AddArc(bounds.Left, bounds.Bottom - d, d, d, 90, 90);
        path.CloseFigure();
        return path;
    }

    public static GraphicsPath CreateBottomRoundedRectanglePath(Rectangle bounds, int radius)
    {
        int r = Math.Max(0, Math.Min(radius, Math.Min(bounds.Width, bounds.Height) / 2));
        var path = new GraphicsPath();
        if (r <= 0)
        {
            path.AddRectangle(bounds);
            return path;
        }

        int d = r * 2;
        path.AddLine(bounds.Left, bounds.Top, bounds.Right, bounds.Top);
        path.AddLine(bounds.Right, bounds.Top, bounds.Right, bounds.Bottom - r);
        path.AddArc(bounds.Right - d, bounds.Bottom - d, d, d, 0, 90);
        path.AddArc(bounds.Left, bounds.Bottom - d, d, d, 90, 90);
        path.AddLine(bounds.Left, bounds.Bottom - r, bounds.Left, bounds.Top);
        path.CloseFigure();
        return path;
    }

    public static void ApplyRoundedRegion(Control control, int radius)
    {
        if (control.Width <= 0 || control.Height <= 0)
            return;

        var path = CreateRoundedRectanglePath(new Rectangle(0, 0, control.Width, control.Height), radius);
        var old = control.Region;
        control.Region = new Region(path);
        old?.Dispose();
        path.Dispose();
    }

    public static void ApplyBottomRoundedRegion(Control control, int radius)
    {
        if (control.Width <= 0 || control.Height <= 0)
            return;

        var path = CreateBottomRoundedRectanglePath(new Rectangle(0, 0, control.Width, control.Height), radius);
        var old = control.Region;
        control.Region = new Region(path);
        old?.Dispose();
        path.Dispose();
    }

    public static void DrawRoundedFill(Graphics g, Rectangle bounds, int radius, Color fill)
    {
        g.SmoothingMode = SmoothingMode.AntiAlias;
        using var path = CreateRoundedRectanglePath(bounds, radius);
        using var brush = new SolidBrush(fill);
        g.FillPath(brush, path);
    }

    public static void DrawRoundedBorder(Graphics g, Rectangle bounds, int radius, Color border, int thickness = 1)
    {
        g.SmoothingMode = SmoothingMode.AntiAlias;
        var inset = new Rectangle(
            bounds.X + thickness / 2,
            bounds.Y + thickness / 2,
            bounds.Width - thickness,
            bounds.Height - thickness);
        using var path = CreateRoundedRectanglePath(inset, radius);
        using var pen = new Pen(border, thickness);
        g.DrawPath(pen, path);
    }

    public static void DrawBottomRoundedBorder(Graphics g, Rectangle bounds, int radius, Color border, int thickness = 1)
    {
        g.SmoothingMode = SmoothingMode.AntiAlias;
        var inset = new Rectangle(
            bounds.X + thickness / 2,
            bounds.Y + thickness / 2,
            bounds.Width - thickness,
            bounds.Height - thickness);
        using var path = CreateBottomRoundedRectanglePath(inset, radius);
        using var pen = new Pen(border, thickness);
        g.DrawPath(pen, path);
    }
}
