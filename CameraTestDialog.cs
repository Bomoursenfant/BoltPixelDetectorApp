using System.Globalization;
using System.Text;
using OpenCvSharp;

namespace BoltPixelDetectorApp;

public sealed class CameraTestDialog : Form
{
    private readonly VisionSettings _settings;
    private readonly DataGridView _grid = new();
    private readonly Label _summary = new();
    private readonly Label _hint = new();
    private List<DetectionResult> _detections = new();
    private HashSet<string> _robotSendKeys = new(StringComparer.Ordinal);
    private BoltSizeStats _sizeStats;
    private Mat? _annotatedFrame;
    private string _timingLine = string.Empty;

    public CameraTestDialog(VisionSettings settings)
    {
        _settings = settings;
        Text = "Test camera";
        Size = new System.Drawing.Size(760, 540);
        MinimumSize = new System.Drawing.Size(580, 420);
        StartPosition = FormStartPosition.CenterParent;
        FormBorderStyle = FormBorderStyle.Sizable;
        MaximizeBox = true;
        UiTheme.ApplyToForm(this);
        BuildLayout();
        RefreshSummaryEmpty();
    }

    public void SetResults(
        IReadOnlyList<DetectionResult> detections,
        BoltSizeStats sizeStats,
        Mat? annotatedFrame,
        IReadOnlySet<string>? robotSendKeys = null,
        string? timingLine = null)
    {
        _detections = detections.ToList();
        _robotSendKeys = robotSendKeys is null
            ? new HashSet<string>(StringComparer.Ordinal)
            : new HashSet<string>(robotSendKeys, StringComparer.Ordinal);
        _sizeStats = sizeStats;
        _timingLine = timingLine ?? string.Empty;
        _annotatedFrame?.Dispose();
        _annotatedFrame = annotatedFrame?.Clone();
        PopulateGrid();
        RefreshSummary();
    }

    private void BuildLayout()
    {
        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 4,
            Padding = new Padding(UiTheme.SpacingLg),
            BackColor = UiTheme.Surface
        };
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 32));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        Controls.Add(root);

        var summaryCard = new CardPanel { Dock = DockStyle.Fill, Title = "Summary" };
        _summary.Dock = DockStyle.Fill;
        _summary.Font = UiTheme.TitleFont;
        _summary.ForeColor = UiTheme.OnSurface;
        summaryCard.Controls.Add(_summary);
        root.Controls.Add(summaryCard, 0, 0);

        _hint.Dock = DockStyle.Fill;
        _hint.ForeColor = UiTheme.OnSurfaceVariant;
        _hint.Font = UiTheme.UiFontSmall;
        _hint.Text = "No robot 0x01, Flask, or database push. Timing (ms) appears in the summary after capture; also on the main status bar.";
        root.Controls.Add(_hint, 0, 1);

        var gridCard = new CardPanel { Dock = DockStyle.Fill, Title = "Results" };
        _grid.Dock = DockStyle.Fill;
        _grid.ReadOnly = true;
        _grid.AllowUserToAddRows = false;
        _grid.AllowUserToDeleteRows = false;
        _grid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
        UiTheme.StyleDataGridView(_grid);
        _grid.Columns.Add("Id", "ID");
        _grid.Columns.Add("X", "X robot");
        _grid.Columns.Add("Y", "Y robot");
        _grid.Columns.Add("Angle", "Angle");
        _grid.Columns.Add("WidthMm", "Width (mm)");
        _grid.Columns.Add("LengthMm", "Length (mm)");
        _grid.Columns.Add("Area", "Area");
        _grid.Columns.Add("Confidence", "Confidence (%)");
        _grid.Columns.Add("M8", "M8?");
        _grid.Columns.Add("Robot", "Send robot?");
        gridCard.Controls.Add(_grid);
        root.Controls.Add(gridCard, 0, 2);

        var actions = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            FlowDirection = FlowDirection.RightToLeft,
            WrapContents = false,
            BackColor = Color.Transparent,
            Padding = new Padding(0, UiTheme.SpacingXs, 0, 0)
        };
        var close = new MaterialButton { Text = "Close", Width = 96, Height = 40 };
        close.Variant = MaterialButtonVariant.Text;
        close.Click += (_, _) => Close();
        var exportImage = new MaterialButton { Text = "Export image", Width = 128, Height = 40 };
        exportImage.Variant = MaterialButtonVariant.Outlined;
        exportImage.Click += (_, _) => ExportImage();
        var export = new MaterialButton { Text = "Export CSV", Width = 120, Height = 40 };
        export.Variant = MaterialButtonVariant.Outlined;
        export.Click += (_, _) => ExportCsv();
        var capture = new MaterialButton { Text = "Capture & detect", Width = 148, Height = 40 };
        capture.Variant = MaterialButtonVariant.Filled;
        capture.Click += (_, _) => OnCaptureRequested?.Invoke(this, EventArgs.Empty);
        actions.Controls.Add(close);
        actions.Controls.Add(exportImage);
        actions.Controls.Add(export);
        actions.Controls.Add(capture);
        root.Controls.Add(actions, 0, 3);
    }

    public event EventHandler? OnCaptureRequested;

    private void RefreshSummaryEmpty()
    {
        _summary.Text = "Click Capture & Detect (camera must be started on main window).";
    }

    private void RefreshSummary()
    {
        var inv = CultureInfo.InvariantCulture;
        string wMin = _sizeStats.WidthMinMm?.ToString("F2", inv) ?? "-";
        string wMax = _sizeStats.WidthMaxMm?.ToString("F2", inv) ?? "-";
        string lMin = _sizeStats.LengthMinMm?.ToString("F2", inv) ?? "-";
        string lMax = _sizeStats.LengthMaxMm?.ToString("F2", inv) ?? "-";
        string areaMin = _sizeStats.AreaMin?.ToString("F0", inv) ?? "-";
        string areaMax = _sizeStats.AreaMax?.ToString("F0", inv) ?? "-";
        string timing = string.IsNullOrWhiteSpace(_timingLine) ? string.Empty : $"  |  {_timingLine.Trim()}";
        _summary.Text =
            $"Objects: {_detections.Count}  |  Width: {wMin}-{wMax} mm  |  Length: {lMin}-{lMax} mm  |  Area: {areaMin}-{areaMax} px^2{timing}  |  M8 width {_settings.M8HeadDiameterMinMm:F1}-{_settings.M8HeadDiameterMaxMm:F1} mm, length {_settings.M8LengthMinMm:F1}-{_settings.M8LengthMaxMm:F1} mm, area {_settings.M8AreaMin:F0}-{_settings.M8AreaMax:F0} px^2";
    }

    private void PopulateGrid()
    {
        _grid.Rows.Clear();
        foreach (var detection in _detections)
        {
            var result = CoordinateMapper.ToVisionResult(detection, _settings);
            string widthText;
            string lengthText;
            string m8Text;
            if (BoltSizeFilter.TryMeasureBoltSizeMm(detection, _settings, out double widthMm, out double lengthMm))
            {
                widthText = widthMm.ToString("F2", CultureInfo.InvariantCulture);
                lengthText = lengthMm.ToString("F2", CultureInfo.InvariantCulture);
                m8Text = BoltSizeFilter.IsWithinM8SizeRange(widthMm, lengthMm, detection.Area, _settings) ? "yes" : "no";
            }
            else
            {
                widthText = "-";
                lengthText = "-";
                m8Text = "-";
            }

            bool sendRobot = _robotSendKeys.Contains(BoltSizeFilter.RobotDetectionKey(detection));
            string robotText = BoltSizeFilter.FormatRobotSendText(sendRobot);

            string confidenceText = VisionSettings.ToConfidencePercent(detection.Confidence)
                .ToString("F1", CultureInfo.InvariantCulture);

            _grid.Rows.Add(
                detection.Id,
                result.X.ToString("F1", CultureInfo.InvariantCulture),
                result.Y.ToString("F1", CultureInfo.InvariantCulture),
                detection.Angle.ToString("F1", CultureInfo.InvariantCulture),
                widthText,
                lengthText,
                detection.Area.ToString("F0", CultureInfo.InvariantCulture),
                confidenceText,
                m8Text,
                robotText);
        }
    }

    private void ExportCsv()
    {
        if (_detections.Count == 0)
        {
            MessageBox.Show(this, "No detections to export.", "Test Camera", MessageBoxButtons.OK, MessageBoxIcon.Information);
            return;
        }

        string exportDir = SystemConfig.EXPORT_CSV_DIRECTORY;
        Directory.CreateDirectory(exportDir);
        string path = Path.Combine(exportDir, $"camera_test_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        const char sep = ';';
        var inv = CultureInfo.InvariantCulture;
        var csv = new StringBuilder();
        csv.AppendLine($"sep={sep}");
        csv.AppendLine(string.Join(sep, new[]
        {
            "id", "x_robot", "y_robot", "angle_degrees", "width_mm", "length_mm", "area",
            "confidence_percent", "m8_in_preview_range", "send_to_robot",
            "width_min_mm", "width_max_mm", "length_min_mm", "length_max_mm", "area_min", "area_max",
            "m8_preview_width_min", "m8_preview_width_max", "m8_preview_length_min", "m8_preview_length_max",
            "m8_preview_area_min", "m8_preview_area_max"
        }));

        string wMin = _sizeStats.WidthMinMm?.ToString("F3", inv) ?? "";
        string wMax = _sizeStats.WidthMaxMm?.ToString("F3", inv) ?? "";
        string lMin = _sizeStats.LengthMinMm?.ToString("F3", inv) ?? "";
        string lMax = _sizeStats.LengthMaxMm?.ToString("F3", inv) ?? "";
        string areaMin = _sizeStats.AreaMin?.ToString("F3", inv) ?? "";
        string areaMax = _sizeStats.AreaMax?.ToString("F3", inv) ?? "";
        foreach (var d in _detections)
        {
            var result = CoordinateMapper.ToVisionResult(d, _settings);
            string widthMm = BoltSizeFilter.TryMeasureBoltSizeMm(d, _settings, out double w, out double len)
                ? w.ToString("F3", inv) : "";
            string lengthMm = BoltSizeFilter.TryMeasureBoltSizeMm(d, _settings, out _, out double l)
                ? l.ToString("F3", inv) : "";
            string m8 = BoltSizeFilter.TryMeasureBoltSizeMm(d, _settings, out double w3, out double l3) &&
                        BoltSizeFilter.IsWithinM8SizeRange(w3, l3, d.Area, _settings) ? "yes"
                : BoltSizeFilter.TryMeasureBoltSizeMm(d, _settings, out _, out _) ? "no" : "";
            string sendRobot = BoltSizeFilter.FormatRobotSendText(
                _robotSendKeys.Contains(BoltSizeFilter.RobotDetectionKey(d)));
            csv.AppendLine(string.Join(sep, new[]
            {
                d.Id.ToString(inv),
                result.X.ToString("F3", inv),
                result.Y.ToString("F3", inv),
                d.Angle.ToString("F3", inv),
                widthMm, lengthMm, d.Area.ToString("F3", inv),
                VisionSettings.ToConfidencePercent(d.Confidence).ToString("F1", inv),
                m8, sendRobot, wMin, wMax, lMin, lMax, areaMin, areaMax,
                _settings.M8HeadDiameterMinMm.ToString("F3", inv),
                _settings.M8HeadDiameterMaxMm.ToString("F3", inv),
                _settings.M8LengthMinMm.ToString("F3", inv),
                _settings.M8LengthMaxMm.ToString("F3", inv),
                _settings.M8AreaMin.ToString("F3", inv),
                _settings.M8AreaMax.ToString("F3", inv)
            }));
        }

        File.WriteAllText(path, csv.ToString(), Encoding.UTF8);
        MessageBox.Show(this, $"Saved:\n{path}", "Test Camera", MessageBoxButtons.OK, MessageBoxIcon.Information);
    }

    private void ExportImage()
    {
        if (_annotatedFrame is null || _annotatedFrame.Empty())
        {
            MessageBox.Show(this, "No annotated image yet. Run Capture & Detect first.", "Test Camera",
                MessageBoxButtons.OK, MessageBoxIcon.Information);
            return;
        }

        string exportDir = SystemConfig.EXPORT_IMAGES_DIRECTORY;
        Directory.CreateDirectory(exportDir);
        string path = Path.Combine(exportDir, $"camera_test_{DateTime.Now:yyyyMMdd_HHmmss}.png");
        _annotatedFrame.SaveImage(path);
        MessageBox.Show(this, $"Saved:\n{path}", "Test Camera", MessageBoxButtons.OK, MessageBoxIcon.Information);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
            _annotatedFrame?.Dispose();
        base.Dispose(disposing);
    }
}
