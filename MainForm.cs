using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using CvPoint = OpenCvSharp.Point;
using Timer = System.Windows.Forms.Timer;

namespace BoltPixelDetectorApp;

public sealed class MainForm : Form
{
    private const int SnapshotDiscardFrames = 3;
    private const int SnapshotDiscardDelayMs = 80;
    
    /// <summary>Horizontal flip applied to grabbed frames before detect, display, save, and robot coords.</summary>
    private static readonly bool MirrorCameraFrame = true;

    private readonly PictureBox _cameraView = new();
    private readonly PictureBox _maskView = new();
    private readonly DataGridView _grid = new();
    private readonly TextBox _sdkRoot = new();
    private readonly NumericUpDown _exposure = new();
    private readonly ComboBox _detectorMode = new();
    private readonly TextBox _yoloModelPath = new();
    private readonly MaterialButton _browseYoloModel = new();
    private readonly TextBox _yoloPythonExe = new();
    private readonly NumericUpDown _yoloImageSize = new();
    private readonly NumericUpDown _yoloNms = new();
    private readonly NumericUpDown _resizeWidth = new();
    private readonly NumericUpDown _resizeHeight = new();
    private readonly NumericUpDown _threshold = new();
    private readonly NumericUpDown _minArea = new();
    private readonly NumericUpDown _maxArea = new();
    private readonly NumericUpDown _minCircularity = new();
    private readonly NumericUpDown _minConfidence = new();
    private readonly NumericUpDown _crosshairLength = new();
    private readonly CheckBox _useRoi = new();
    private readonly NumericUpDown _roiX = new();
    private readonly NumericUpDown _roiY = new();
    private readonly NumericUpDown _roiWidth = new();
    private readonly NumericUpDown _roiHeight = new();
    private readonly CheckBox _invert = new();
    private readonly CheckBox _showMask = new();
    private readonly MaterialButton _settingVision = new();
    private readonly MaterialButton _settingRobot = new();
    private readonly MaterialButton _startStop = new();
    private readonly MaterialButton _saveCsv = new();
    private readonly MaterialButton _saveImage = new();
    private readonly MaterialButton _testCamera = new();
    private readonly Label _integrationStatus = new();
    private readonly Label _integrationHint = new();
    private readonly Label _status = new();
    private readonly Timer _timer = new();
    private readonly ToolTip _toolTip = new();
    private readonly BoltDetector _detector = new();
    private readonly YoloSegmentationDetector _yoloDetector = new();
    private readonly VisionSettings _visionSettings = VisionSettings.Load();
    private readonly RobotConnectionSettings _robotSettings = RobotConnectionSettings.Load();

    private NeptuneCamera? _neptune;
    private Mat? _lastAnnotatedFrame;
    private List<DetectionResult> _lastDetections = new();
    /// <summary>All detections before M8 filter (for calibration grid / overlay).</summary>
    private List<DetectionResult> _lastAllDetections = new();
    private bool _isProcessingFrame;
    private int _snapshotIndex;
    private FlaskApiClient? _flaskClient;
    private VisionDatabase? _database;
    private RobotComms? _robot;
    private bool _flaskConnected;
    /// <summary>True while Test Camera capture runs — blocks Flask/DB writes.</summary>
    private bool _integrationWritesBlocked;
    private List<DetectionResult> _testCameraAllDetections = new();
    private HashSet<string> _lastRobotSendKeys = new(StringComparer.Ordinal);
    private CameraTestDialog? _cameraTestDialog;
    private SceneBinaryContext? _lastSceneContext;
    private FrameTiming _lastFrameTiming;
    private int _lastRobotM8Total;
    private string _pendingStatusWithoutTiming = string.Empty;

    private static readonly Regex TimingSuffixRegex = new(@" Detect \d+ ms \|.*$", RegexOptions.Compiled);

    private readonly struct FrameTiming
    {
        public long DetectMs { get; init; }
        public long DbMs { get; init; }
        public long RobotFetchMs { get; init; }
        public long TotalMs { get; init; }
    }

    public MainForm()
    {
        Text = "Bolt Pixel Detector";
        Width = 1320;
        Height = 860;
        MinimumSize = new System.Drawing.Size(1080, 720);
        StartPosition = FormStartPosition.CenterScreen;
        UiTheme.ApplyToForm(this);
        _visionSettings.ApplyToSystemConfig();

        BuildLayout();
        ApplyChrome();
        ConfigureTimer();
        ConfigureExportToolTips();
        FormClosing += (_, _) => StopCamera();

        // Khởi động trước model YOLO ở luồng chính để tránh bị chậm ở lần chụp đầu
        try
        {
            if (!string.IsNullOrWhiteSpace(_visionSettings.YoloModelPath) && File.Exists(_visionSettings.YoloModelPath))
            {
                _yoloDetector.Warmup(_visionSettings.YoloModelPath, _visionSettings.YoloImageSize);
            }
        }
        catch
        {
            // Bỏ qua nếu lỗi
        }
    }

    private void ApplyChrome()
    {
        ConfigureGrid();
        UiTheme.StyleScrollableDataGridView(_grid);
        ApplyDetectionColumnWidths();
        UiTheme.StyleIntegrationStatus(_integrationStatus);
        _integrationStatus.BackColor = Color.Transparent;
        UiTheme.StyleStatusBar(_status);

        _settingVision.Text = "Vision Settings";
        _settingVision.Variant = MaterialButtonVariant.Outlined;

        _settingRobot.Text = "Robot Settings";
        _settingRobot.Variant = MaterialButtonVariant.Outlined;

        _saveCsv.Variant = MaterialButtonVariant.Outlined;
        _saveImage.Variant = MaterialButtonVariant.FilledTonal;

        _startStop.Text = "Start";
        _startStop.Variant = MaterialButtonVariant.Filled;

        _testCamera.Text = "Test";
        _testCamera.Variant = MaterialButtonVariant.FilledTonal;
    }

    private void BuildLayout()
    {
        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 2,
            Padding = new Padding(UiTheme.SpacingMd),
            BackColor = UiTheme.Surface
        };
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 68));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 32));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));
        Controls.Add(root);

        var cameraFrame = new RoundedPanel
        {
            Dock = DockStyle.Fill,
            CornerRadius = UiTheme.RadiusXLarge,
            FillColor = UiTheme.ViewportBackground
        };
        _cameraView.Dock = DockStyle.Fill;
        _cameraView.BackColor = UiTheme.ViewportBackground;
        _cameraView.SizeMode = PictureBoxSizeMode.Zoom;
        cameraFrame.Controls.Add(_cameraView);
        root.Controls.Add(cameraFrame, 0, 0);

        var side = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 4,
            Padding = new Padding(UiTheme.SpacingSm, 0, 0, 0),
            BackColor = UiTheme.Surface
        };
        side.RowStyles.Add(new RowStyle(SizeType.Absolute, 168));
        side.RowStyles.Add(new RowStyle(SizeType.Absolute, 92));
        side.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        side.RowStyles.Add(new RowStyle(SizeType.Absolute, 120));
        root.Controls.Add(side, 1, 0);

        var actionsCard = new CardPanel
        {
            Dock = DockStyle.Fill,
            Title = "Actions",
            TitleBottomGap = UiTheme.SpacingMd
        };
        var actionPanel = BuildControls();
        actionPanel.Dock = DockStyle.Fill;
        actionsCard.Controls.Add(actionPanel);
        side.Controls.Add(actionsCard, 0, 0);

        var integrationCard = new CardPanel { Dock = DockStyle.Fill, Title = "Integration" };
        var integrationBody = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            BackColor = Color.Transparent
        };
        integrationBody.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
        integrationBody.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        _integrationHint.Dock = DockStyle.Fill;
        _integrationHint.TextAlign = ContentAlignment.TopLeft;
        _integrationHint.Text = "Start opens the camera only. Save image captures manually. Robot 0x01 captures and detects live.";
        _integrationHint.ForeColor = UiTheme.OnSurfaceVariant;
        _integrationHint.Font = UiTheme.UiFontSmall;
        _integrationHint.BackColor = Color.Transparent;
        _integrationHint.AutoEllipsis = true;

        _integrationStatus.Dock = DockStyle.Fill;
        _integrationStatus.TextAlign = ContentAlignment.MiddleLeft;
        _integrationStatus.AutoEllipsis = true;
        _integrationStatus.Text = "Robot / API / DB: not started.";
        _integrationStatus.BackColor = Color.Transparent;

        integrationBody.Controls.Add(_integrationHint, 0, 0);
        integrationBody.Controls.Add(_integrationStatus, 0, 1);
        integrationCard.Controls.Add(integrationBody);
        side.Controls.Add(integrationCard, 0, 1);

        var gridCard = new CardPanel { Dock = DockStyle.Fill, Title = "Detections" };
        _grid.Dock = DockStyle.Fill;
        gridCard.Controls.Add(_grid);
        side.Controls.Add(gridCard, 0, 2);

        var maskFrame = new RoundedPanel
        {
            Dock = DockStyle.Fill,
            CornerRadius = UiTheme.RadiusLarge,
            FillColor = UiTheme.ViewportBackground
        };
        _maskView.Dock = DockStyle.Fill;
        _maskView.BackColor = UiTheme.ViewportBackground;
        _maskView.SizeMode = PictureBoxSizeMode.Zoom;
        maskFrame.Controls.Add(_maskView);
        var maskCard = new CardPanel { Dock = DockStyle.Fill, Title = "Mask preview" };
        maskCard.Controls.Add(maskFrame);
        maskFrame.Dock = DockStyle.Fill;
        side.Controls.Add(maskCard, 0, 3);

        var statusBar = new RoundedPanel
        {
            Dock = DockStyle.Fill,
            CornerRadius = UiTheme.RadiusMedium,
            FillColor = UiTheme.SurfaceContainerHigh,
            BorderWidth = 0
        };
        _status.Dock = DockStyle.Fill;
        _status.TextAlign = ContentAlignment.MiddleLeft;
        _status.Text = "Ready.";
        _status.AutoEllipsis = true;
        statusBar.Controls.Add(_status);
        root.SetColumnSpan(statusBar, 2);
        root.Controls.Add(statusBar, 0, 1);
    }

    private Control BuildControls()
    {
        const int buttonHeight = 30;
        var controls = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 3,
            BackColor = UiTheme.SurfaceContainer
        };
        controls.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        controls.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        for (int i = 0; i < 3; i++)
            controls.RowStyles.Add(new RowStyle(SizeType.Percent, 33.33f));

        void StyleActionButton(MaterialButton button)
        {
            button.Dock = DockStyle.Fill;
            button.Height = buttonHeight;
            button.MinimumSize = new System.Drawing.Size(72, buttonHeight);
            button.Font = UiTheme.UiFont;
            button.Margin = new Padding(2);
        }

        _settingVision.Click += (_, _) => OpenVisionSettings();
        _settingRobot.Click += (_, _) => OpenRobotSettings();
        _saveCsv.Text = "Save CSV";
        _saveCsv.Click += (_, _) => SaveCsv();
        _saveImage.Text = "Save image";
        _saveImage.Click += (_, _) => SaveImage();
        _startStop.Click += (_, _) => ToggleCamera();
        _testCamera.Click += (_, _) => OpenTestCameraDialog();

        StyleActionButton(_settingVision);
        StyleActionButton(_settingRobot);
        StyleActionButton(_saveCsv);
        StyleActionButton(_saveImage);
        StyleActionButton(_startStop);
        StyleActionButton(_testCamera);

        controls.Controls.Add(_settingVision, 0, 0);
        controls.Controls.Add(_settingRobot, 1, 0);
        controls.Controls.Add(_saveCsv, 0, 1);
        controls.Controls.Add(_saveImage, 1, 1);
        controls.Controls.Add(_startStop, 0, 2);
        controls.Controls.Add(_testCamera, 1, 2);

        return controls;
    }

    private static void ConfigureDimensionInput(NumericUpDown input, decimal min, decimal max, decimal value, decimal increment)
    {
        input.Minimum = min;
        input.Maximum = max;
        input.Value = value;
        input.Increment = increment;
    }

    private static void AddRow(TableLayoutPanel panel, int row, string label, Control control)
    {
        panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 28));

        var text = new Label
        {
            Text = label,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft
        };

        control.Dock = DockStyle.Fill;
        panel.Controls.Add(text, 0, row);
        panel.Controls.Add(control, 1, row);
    }

    private void BrowseYoloModel()
    {
        using var dialog = new OpenFileDialog
        {
            Title = "Select YOLOv8 segmentation model",
            Filter = "YOLO model (*.onnx;*.pt)|*.onnx;*.pt|ONNX model (*.onnx)|*.onnx|PyTorch checkpoint (*.pt)|*.pt|All files (*.*)|*.*",
            CheckFileExists = true,
            Multiselect = false
        };

        string currentPath = _yoloModelPath.Text.Trim();
        string? currentDirectory = Path.GetDirectoryName(currentPath);
        if (!string.IsNullOrWhiteSpace(currentDirectory) && Directory.Exists(currentDirectory))
            dialog.InitialDirectory = currentDirectory;
        else if (Directory.Exists(AppContext.BaseDirectory))
            dialog.InitialDirectory = AppContext.BaseDirectory;

        if (dialog.ShowDialog(this) == DialogResult.OK)
        {
            _yoloModelPath.Text = dialog.FileName;
            SystemConfig.YOLO_MODEL_PATH = dialog.FileName;
        }
    }

    private void ConfigureGrid()
    {
        _grid.Dock = DockStyle.Fill;
        _grid.AllowUserToAddRows = false;
        _grid.AllowUserToDeleteRows = false;
        _grid.ReadOnly = true;
        _grid.SelectionMode = DataGridViewSelectionMode.FullRowSelect;
        _grid.Columns.Add("Id", "ID");
        _grid.Columns.Add("WidthMm", "Width (mm)");
        _grid.Columns.Add("LengthMm", "Length (mm)");
        _grid.Columns.Add("M8", "M8?");
        _grid.Columns.Add("Robot", "Send robot?");
        _grid.Columns.Add("X", "X robot");
        _grid.Columns.Add("Y", "Y robot");
        _grid.Columns.Add("Angle", "Angle");
        _grid.Columns.Add("Area", "Area");
        _grid.Columns.Add("Confidence", "Confidence (%)");
        _grid.Columns.Add("Circularity", "Circularity");
    }

    private void ApplyDetectionColumnWidths()
    {
        SetDetectionColumn("Id", "ID", 52);
        SetDetectionColumn("WidthMm", "Width (mm)", UiTheme.MeasureHeaderMinWidth("Width (mm)"));
        SetDetectionColumn("LengthMm", "Length (mm)", UiTheme.MeasureHeaderMinWidth("Length (mm)"));
        SetDetectionColumn("M8", "M8?", 56);
        SetDetectionColumn("Robot", "Send robot?", UiTheme.MeasureHeaderMinWidth("Send robot?"));
        SetDetectionColumn("X", "X robot", UiTheme.MeasureHeaderMinWidth("X robot"));
        SetDetectionColumn("Y", "Y robot", UiTheme.MeasureHeaderMinWidth("Y robot"));
        SetDetectionColumn("Angle", "Angle", 72);
        SetDetectionColumn("Area", "Area", 72);
        SetDetectionColumn("Confidence", "Confidence (%)", UiTheme.MeasureHeaderMinWidth("Confidence (%)"));
        SetDetectionColumn("Circularity", "Circularity", UiTheme.MeasureHeaderMinWidth("Circularity"));
    }

    private void SetDetectionColumn(string name, string header, int width)
    {
        if (!_grid.Columns.Contains(name))
            return;

        var column = _grid.Columns[name];
        column.HeaderText = header;
        column.MinimumWidth = width;
        column.Width = width;
        column.SortMode = DataGridViewColumnSortMode.NotSortable;
        column.DefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleCenter;
        column.HeaderCell.Style.Alignment = DataGridViewContentAlignment.MiddleCenter;
        column.HeaderCell.Style.WrapMode = DataGridViewTriState.True;
    }

    private void ConfigureTimer()
    {
        _timer.Interval = 60_000;
    }

    private void ConfigureExportToolTips()
    {
        _toolTip.SetToolTip(_settingVision, "Configure camera, detector, ROI, confidence, and display settings.");
        _toolTip.SetToolTip(_settingRobot, "Configure robot socket, Flask API, and database settings.");
        _toolTip.SetToolTip(_saveCsv, $"Export detection table to CSV:\n{SystemConfig.EXPORT_CSV_DIRECTORY}");
        _toolTip.SetToolTip(_saveImage, $"Capture from camera, detect bolts, and save annotated PNG:\n{SystemConfig.EXPORT_IMAGES_DIRECTORY}");
        _toolTip.SetToolTip(_startStop, "Open or close the Neptune camera and robot socket (no automatic capture).");
        _toolTip.SetToolTip(_testCamera, "Capture, detect, and show results without robot 0x01 or Flask/DB.");
    }

    private void OpenTestCameraDialog()
    {
        if (_neptune is null || !_neptune.IsOpened)
        {
            _status.Text = "Test Camera: press Start System to open the camera first.";
            return;
        }

        if (_cameraTestDialog is null || _cameraTestDialog.IsDisposed)
        {
            _cameraTestDialog = new CameraTestDialog(_visionSettings);
            _cameraTestDialog.OnCaptureRequested += (_, _) => RunTestCameraCapture();
            _cameraTestDialog.FormClosed += (_, _) => _cameraTestDialog = null;
            _cameraTestDialog.Show(this);
        }
        else
        {
            _cameraTestDialog.BringToFront();
            _cameraTestDialog.Focus();
        }
    }

    private void RunTestCameraCapture()
    {
        if (_neptune is null || !_neptune.IsOpened)
        {
            _status.Text = "Test Camera: camera is not open.";
            return;
        }

        if (_isProcessingFrame)
        {
            _status.Text = "Test Camera: previous capture still running.";
            return;
        }

        _isProcessingFrame = true;
        _integrationWritesBlocked = true;
        _testCameraAllDetections = new List<DetectionResult>();
        _lastRobotSendKeys = new HashSet<string>(StringComparer.Ordinal);
        var swTestTotal = Stopwatch.StartNew();
        try
        {
            var grabbed = GrabFreshFrame(_neptune);
            if (grabbed is null || grabbed.Empty())
            {
                grabbed?.Dispose();
                _status.Text = "Test Camera: no frame from Neptune camera.";
                _cameraTestDialog?.SetResults(
                    Array.Empty<DetectionResult>(),
                    default,
                    null,
                    _lastRobotSendKeys,
                    string.Empty);
                return;
            }

            int snapshotIndex = ++_snapshotIndex;
            Mat? testPreview = null;
            using (grabbed)
            {
                using var frame = PrepareCameraFrame(grabbed);
                testPreview = ProcessFrame(
                    frame,
                    autoSave: false,
                    snapshotIndex: snapshotIndex,
                    pushToIntegrations: false,
                    deferTimingAppend: true);
            }

            var sizeStats = BoltSizeFilter.ComputeBoltSizeStats(_testCameraAllDetections, _visionSettings);
            _lastFrameTiming = _lastFrameTiming with { TotalMs = swTestTotal.ElapsedMilliseconds };
            string timingLine = FormatTimingLine(_lastFrameTiming, includeRobotFetch: false).Trim();
            _cameraTestDialog?.SetResults(_testCameraAllDetections, sizeStats, testPreview, _lastRobotSendKeys, timingLine);
            testPreview?.Dispose();
            ApplyTimingToStatus(includeRobotFetch: false);
        }
        catch (Exception ex)
        {
            _status.Text = $"Test Camera error: {ex.Message}";
            _cameraTestDialog?.SetResults(
                Array.Empty<DetectionResult>(),
                default,
                null,
                _lastRobotSendKeys,
                string.Empty);
        }
        finally
        {
            _integrationWritesBlocked = false;
            _isProcessingFrame = false;
        }
    }

    private void OpenVisionSettings()
    {
        try
        {
            using var dialog = new VisionSettingsDialog(_visionSettings);
            if (dialog.ShowDialog(this) == DialogResult.OK)
            {
                _neptune?.SetExposure(_visionSettings.ExposureUs);
                _status.Text = "Vision settings saved.";
            }
        }
        catch (Exception ex)
        {
            _status.Text = $"Vision settings error: {ex.Message}";
            MessageBox.Show(this, ex.Message, "Vision settings", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    private void OpenRobotSettings()
    {
        using var dialog = new RobotSettingsDialog(_robotSettings);
        if (dialog.ShowDialog(this) == DialogResult.OK)
        {
            ResetIntegrationClients();
            if (_neptune is not null)
                StartRobotIntegration();
            _status.Text = "Robot, Flask API, and Database settings saved.";
        }
    }

    private void ToggleCamera()
    {
        if (_neptune is null)
        {
            StartCamera();
        }
        else
        {
            StopCamera();
        }
    }

    private void StartCamera()
    {
        _visionSettings.ApplyToSystemConfig();
        SystemConfig.NEPTUNE_SDK_ROOT = _visionSettings.SdkRoot;
        if (!Directory.Exists(SystemConfig.NEPTUNE_SDK_ROOT))
        {
            _status.Text = $"Neptune SDK root does not exist: {SystemConfig.NEPTUNE_SDK_ROOT}";
            return;
        }

        var neptune = new NeptuneCamera();
        try
        {
            if (!neptune.Open())
            {
                neptune.Dispose();
                _status.Text = "Cannot open Neptune camera. Check camera connection and IMI Tech SDK.";
                return;
            }

            neptune.SetExposure(_visionSettings.ExposureUs);
            _neptune = neptune;
            _snapshotIndex = 0;
            StartRobotIntegration();
            _startStop.Text = "Stop";
            _status.Text = "System started. Use Save image to capture and detect.";
        }
        catch (Exception ex)
        {
            neptune.Dispose();
            _status.Text = $"Neptune startup error: {ex.Message}";
        }
    }

    private void StopCamera()
    {
        _timer.Stop();
        if (_cameraTestDialog is not null && !_cameraTestDialog.IsDisposed)
            _cameraTestDialog.Close();
        _cameraTestDialog = null;
        StopRobotIntegration();
        _neptune?.Close();
        _neptune?.Dispose();
        _neptune = null;
        _snapshotIndex = 0;
        _startStop.Text = "Start";
        _status.Text = "Neptune camera stopped.";
    }

    private Mat? GrabFreshFrame(NeptuneCamera camera)
    {
        for (int i = 0; i < SnapshotDiscardFrames; i++)
        {
            using var staleFrame = camera.GrabFrame();
            Thread.Sleep(SnapshotDiscardDelayMs);
        }

        return camera.GrabFrame(timeoutMs: 2000);
    }

    private static Mat PrepareCameraFrame(Mat grabbed)
    {
        if (!MirrorCameraFrame)
            return grabbed;

        var mirrored = new Mat();
        Cv2.Flip(grabbed, mirrored, FlipMode.Y);
        return mirrored;
    }

    private void StartRobotIntegration()
    {
        EnsureIntegrationClients();
        StopRobotIntegration();

        _robot = new RobotComms(_robotSettings);
        _robot.FetchRobotCoordinateBatchAsync = CaptureRobotBatchOnRequestAsync;
        _robot.FetchPendingRobotCoordinateCountAsync = FetchPendingRobotCoordinateCountAsync;
        _robot.SaveCycleTimingAsync = SaveCycleTimingAsync;
        _robot.OnRobotDataReceived += (_, data) => BeginInvoke(new Action(() =>
        {
            string lastReq = data.LastRequestCode is >= 0 and <= 255
                ? $"0x{data.LastRequestCode:X2}"
                : "none";
            _integrationStatus.Text =
                $"Robot/API/DB: {data.Status} | Pending={data.PendingCount} | Flask={(_flaskConnected ? "OK" : "OFF/ERR")} | Last robot req={lastReq}";
        }));

        bool connected = _robot.Connect();
        _integrationStatus.Text = connected
            ? $"Robot/API/DB: socket ready on {_robotSettings.PcListenIP}:{_robotSettings.PcListenPort}"
            : "Robot/API/DB: socket failed.";
    }

    private void StopRobotIntegration()
    {
        _robot?.Dispose();
        _robot = null;
        _integrationStatus.Text = "Robot/API/DB: stopped.";
    }

    private Task<RobotCoordinateBatch?> CaptureRobotBatchOnRequestAsync(int maxItems)
    {
        if (IsDisposed || !IsHandleCreated)
            return Task.FromResult<RobotCoordinateBatch?>(BuildEmptyRobotBatch("UI not ready"));

        var tcs = new TaskCompletionSource<RobotCoordinateBatch?>(
            TaskCreationOptions.RunContinuationsAsynchronously);

        void Capture()
        {
            try
            {
                tcs.SetResult(CaptureRobotBatchOnUiThread(maxItems));
            }
            catch (Exception ex)
            {
                _status.Text = $"Robot-triggered snapshot error: {ex.Message}";
                tcs.SetResult(BuildEmptyRobotBatch($"Capture failed: {ex.Message}"));
            }
        }

        if (InvokeRequired)
            BeginInvoke(new Action(Capture));
        else
            Capture();

        return tcs.Task;
    }

    private RobotCoordinateBatch CaptureRobotBatchOnUiThread(int maxItems)
    {
        if (_neptune is null || !_neptune.IsOpened)
            return BuildEmptyRobotBatch("Camera not ready");

        if (_isProcessingFrame)
            return BuildEmptyRobotBatch("Capture in progress");

        _isProcessingFrame = true;
        var swTotal = Stopwatch.StartNew();
        try
        {
            var grabbed = GrabFreshFrame(_neptune);
            if (grabbed is null || grabbed.Empty())
            {
                grabbed?.Dispose();
                return BuildEmptyRobotBatch("No frame from camera");
            }

            int snapshotIndex = ++_snapshotIndex;
            _lastDetections = new List<DetectionResult>();
            _lastAllDetections = new List<DetectionResult>();
            _lastRobotM8Total = 0;
            using (grabbed)
            {
                using var frame = PrepareCameraFrame(grabbed);
                ProcessFrame(
                    frame,
                    autoSave: false,
                    snapshotIndex: snapshotIndex,
                    pushToIntegrations: true,
                    deferTimingAppend: true);
            }

            var swFetch = Stopwatch.StartNew();
            RobotCoordinateBatch? batch = FetchRobotCoordinateBatchAsync(maxItems).GetAwaiter().GetResult();
            swFetch.Stop();

            _lastFrameTiming = _lastFrameTiming with
            {
                RobotFetchMs = swFetch.ElapsedMilliseconds,
                TotalMs = swTotal.ElapsedMilliseconds
            };

            ApplyTimingToStatus(includeRobotFetch: true);

            RobotCoordinateBatch finalBatch = batch ?? BuildEmptyRobotBatch("No DB/Flask queue target");
            finalBatch.TotalBeforeSend = _lastRobotM8Total;
            finalBatch.Timing = CreateCycleTiming(
                snapshotIndex,
                "Robot TCP",
                finalBatch.Source,
                _lastFrameTiming,
                notes: "Robot-triggered capture; TCP timing is filled after reply bytes are written.");
            return finalBatch;
        }
        finally
        {
            _isProcessingFrame = false;
        }
    }

    private static RobotCoordinateBatch BuildEmptyRobotBatch(string source)
    {
        return new RobotCoordinateBatch
        {
            Results = new List<VisionResult>(),
            TotalBeforeSend = 0,
            Source = source
        };
    }

    private async Task<RobotCoordinateBatch?> FetchRobotCoordinateBatchAsync(int maxItems)
    {
        if (_robotSettings.EnableFlaskApi && _flaskClient is not null && _flaskConnected)
        {
            int total = (await _flaskClient.GetStatistics().ConfigureAwait(false))?.PendingRobotCoordinates ?? 0;
            var results = new List<VisionResult>();
            for (int i = 0; i < maxItems; i++)
            {
                var pending = await _flaskClient.GetPendingRobotCoordinate(markSent: true).ConfigureAwait(false);
                if (pending is null) break;
                results.Add(pending.ToVisionResult());
            }

            // Robot dequeue via Flask: mirror FIFO sent flags on local SQLite when dual-write is on.
            if (_robotSettings.EnableLocalDatabase && _database is not null && results.Count > 0)
                _database.MarkOldestPendingRobotCoordinatesSent(results.Count);

            return new RobotCoordinateBatch { Results = results, TotalBeforeSend = total, Source = "Flask/DB" };
        }

        if (_robotSettings.EnableLocalDatabase && _database is not null)
        {
            int total = _database.CountPendingRobotCoordinates();
            var results = _database.GetPendingRobotCoordinates(maxItems, markSent: true);
            return new RobotCoordinateBatch { Results = results, TotalBeforeSend = total, Source = "Local DB" };
        }

        return null;
    }

    private Task<int?> FetchPendingRobotCoordinateCountAsync() =>
        Task.FromResult<int?>(_lastRobotM8Total);

    /// <returns>Annotated preview for Test Camera export; null for production path.</returns>
    private Mat? ProcessFrame(
        Mat frame,
        bool autoSave,
        int snapshotIndex,
        bool pushToIntegrations = true,
        bool deferTimingAppend = false)
    {
        var swTotal = Stopwatch.StartNew();
        var swDetect = Stopwatch.StartNew();
        var roi = BuildDetectionRoi(frame);
        using var detectFrame = frame.Clone();
        List<DetectionResult> detections;
        Mat mask;

        if (_visionSettings.DetectorMode == 1)
        {
            _visionSettings.ApplyToSystemConfig();

            try
            {
                using var sceneBinary = _detector.BuildBinaryMask(
                    detectFrame,
                    _visionSettings.Threshold,
                    _visionSettings.InvertBinary,
                    roi);
                RefreshSceneBinaryContext(sceneBinary);
                detections = _yoloDetector.Detect(
                    detectFrame,
                    SystemConfig.YOLO_MODEL_PATH,
                    SystemConfig.YOLO_PYTHON_EXE,
                    SystemConfig.YOLO_IMAGE_SIZE,
                    SystemConfig.YOLO_NMS_THRESHOLD,
                    _visionSettings.MinConfidencePercent / 100.0,
                    roi,
                    out mask);
            }
            catch (Exception ex)
            {
                _status.Text = $"YOLO segmentation error: {ex.Message}";
                return null;
            }
        }
        else if (_visionSettings.DetectorMode == 2)
        {
            _visionSettings.ApplyToSystemConfig();

            try
            {
                using var opencvBinary = _detector.BuildBinaryMask(
                    detectFrame,
                    _visionSettings.Threshold,
                    _visionSettings.InvertBinary,
                    roi);
                RefreshSceneBinaryContext(opencvBinary);
                detections = _yoloDetector.DetectFused(
                    detectFrame,
                    SystemConfig.YOLO_MODEL_PATH,
                    SystemConfig.YOLO_PYTHON_EXE,
                    SystemConfig.YOLO_IMAGE_SIZE,
                    SystemConfig.YOLO_NMS_THRESHOLD,
                    _visionSettings.MinConfidencePercent / 100.0,
                    roi,
                    opencvBinary,
                    _visionSettings.MinArea,
                    SystemConfig.FUSION_MIN_AREA_RATIO,
                    out mask);
            }
            catch (Exception ex)
            {
                _status.Text = $"YOLO+OpenCV fusion error: {ex.Message}";
                return null;
            }
        }
        else
        {
            detections = _detector.Detect(
                detectFrame,
                _visionSettings.Threshold,
                _visionSettings.MinArea,
                _visionSettings.MaxArea,
                _visionSettings.MinCircularity,
                _visionSettings.MinConfidencePercent / 100.0,
                _visionSettings.InvertBinary,
                roi,
                out mask);
            RefreshSceneBinaryContext(mask);
        }

        var roiDetections = roi.HasValue ? FilterDetectionsByRoi(detections, roi.Value) : detections;
        roiDetections = roiDetections
            .Where(d => d.Area >= _visionSettings.MinArea && d.Area <= _visionSettings.MaxArea)
            .ToList();
        var allDetections = ProjectDetectionsToWorkFrame(roiDetections, frame.Height);
        int countBeforeM8Filter = allDetections.Count;
        var filteredDetections = BoltSizeFilter.ApplyM8HeadFilter(allDetections, _visionSettings);
        int robotSafetyRejected = 0;
        List<DetectionResult> robotReadyDetections;
        try
        {
            robotReadyDetections = ApplyRobotSafetyFilter(allDetections, filteredDetections, out robotSafetyRejected);
            _lastRobotSendKeys = new HashSet<string>(
                robotReadyDetections.Select(BoltSizeFilter.RobotDetectionKey),
                StringComparer.Ordinal);
        }
        catch (Exception ex)
        {
            robotReadyDetections = new List<DetectionResult>();
            _lastRobotSendKeys = new HashSet<string>(StringComparer.Ordinal);
            _status.Text = $"Robot safety (B3) error: {ex.Message}";
        }

        swDetect.Stop();
        long detectMs = swDetect.ElapsedMilliseconds;

        if (pushToIntegrations)
        {
            _lastAllDetections = allDetections;
            _lastDetections = robotReadyDetections;
            _lastRobotM8Total = filteredDetections.Count;
        }
        else
        {
            _testCameraAllDetections = allDetections;
        }

        long dbMs = 0;
        string integrationStatus = string.Empty;
        if (pushToIntegrations)
        {
            var swDb = Stopwatch.StartNew();
            integrationStatus = PushDetectionsToIntegrations(robotReadyDetections);
            dbMs = swDb.ElapsedMilliseconds;
        }

        Mat? testAnnotatedClone = null;
        using (mask)
        {
            var annotated = frame.Clone();
            DrawDetectionFrame(annotated, roi);
            DrawDetections(annotated, allDetections);
            using var display = BuildDisplayFrame(annotated, _cameraView);
            ShowMat(_cameraView, display);

            if (pushToIntegrations)
            {
                _lastAnnotatedFrame?.Dispose();
                _lastAnnotatedFrame = annotated.Clone();
            }
            else
            {
                testAnnotatedClone = annotated.Clone();
            }

            annotated.Dispose();

            if (_visionSettings.ShowMask)
            {
                using var displayMask = BuildDisplayMask(mask, frame.Size(), roi);
                using var maskBgr = new Mat();
                Cv2.CvtColor(displayMask, maskBgr, ColorConversionCodes.GRAY2BGR);
                using var maskDisplay = BuildDisplayFrame(maskBgr, _maskView);
                ShowMat(_maskView, maskDisplay);
            }
            else
            {
                ClearPicture(_maskView);
            }
        }

        UpdateGrid(allDetections);
        string roiText = roi.HasValue
            ? $" ROI=({roi.Value.X},{roi.Value.Y},{roi.Value.Width},{roi.Value.Height})."
            : " Full frame.";
        string capturedAt = DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture);
        string m8FilterText = BuildM8StatusText(countBeforeM8Filter, filteredDetections.Count);
        string robotSafetyText = pushToIntegrations
            ? BuildRobotSafetyStatusText(filteredDetections.Count, robotReadyDetections.Count, robotSafetyRejected)
            : string.Empty;

        int shownCount = pushToIntegrations
            ? robotReadyDetections.Count
            : _visionSettings.EnableM8SizeFilter ? filteredDetections.Count : countBeforeM8Filter;
        var sizeStats = BoltSizeFilter.ComputeBoltSizeStats(allDetections, _visionSettings);
        string headRangeText = FormatBoltSizeRangeSummary(sizeStats);
        string prefix = pushToIntegrations ? "Snapshot" : "Test camera";
        string status = $"{prefix} #{snapshotIndex} at {capturedAt}. Detected {shownCount} object(s) (grid lists all {countBeforeM8Filter} with Width/Length mm).{headRangeText} ROI is crop-only, no image scaling. Origin is image bottom-left: X=up, Y=right, Angle=clockwise.{roiText}{m8FilterText}{robotSafetyText}";
        if (!pushToIntegrations)
            status += " No robot/Flask/DB (not saved to vision_results.db).";
        if (!string.IsNullOrWhiteSpace(integrationStatus))
            status += $" {integrationStatus}";
        if (autoSave && pushToIntegrations && _lastAnnotatedFrame is not null)
        {
            string savedPath = SaveAnnotatedImage();
            status += $" Saved image: {Path.GetFullPath(savedPath)}";
        }

        _lastFrameTiming = new FrameTiming
        {
            DetectMs = detectMs,
            DbMs = dbMs,
            RobotFetchMs = 0,
            TotalMs = swTotal.ElapsedMilliseconds
        };

        if (deferTimingAppend)
            _pendingStatusWithoutTiming = status;
        else
        {
            _status.Text = status + FormatTimingLine(_lastFrameTiming, includeRobotFetch: false);
            _ = SaveCycleTimingAsync(CreateCycleTiming(
                snapshotIndex,
                autoSave ? "Manual Save image" : "Manual",
                pushToIntegrations ? "Integration queue" : "None",
                _lastFrameTiming,
                notes: "Manual capture; no robot TCP request/reply in this cycle."));
        }

        return testAnnotatedClone;
    }

    private VisionCycleTiming CreateCycleTiming(
        int snapshotIndex,
        string triggerSource,
        string queueSource,
        FrameTiming timing,
        string notes = "")
    {
        return new VisionCycleTiming
        {
            SnapshotIndex = snapshotIndex,
            TriggerSource = triggerSource,
            DetectionCount = _lastAllDetections.Count,
            RobotReadyCount = _lastDetections.Count,
            DetectMs = timing.DetectMs,
            DbFlaskMs = timing.DbMs,
            RobotFetchMs = timing.RobotFetchMs,
            TotalCycleMs = timing.TotalMs,
            QueueSource = queueSource,
            Notes = notes
        };
    }

    private Task SaveCycleTimingAsync(VisionCycleTiming timing)
    {
        try
        {
            if (_integrationWritesBlocked)
                return Task.CompletedTask;

            if (_robotSettings.EnableFlaskApi && _flaskClient is not null && _flaskConnected)
                _flaskClient.SendCycleTiming(timing).GetAwaiter().GetResult();

            if (_robotSettings.EnableLocalDatabase && _database is not null)
                _database.SaveCycleTiming(timing);
        }
        catch
        {
            // Timing persistence is diagnostic only; it must not affect vision or robot flow.
        }

        return Task.CompletedTask;
    }

    private static string FormatTimingLine(FrameTiming timing, bool includeRobotFetch)
    {
        if (includeRobotFetch)
            return $" Detect {timing.DetectMs} ms | DB {timing.DbMs} ms | Robot fetch {timing.RobotFetchMs} ms | Total {timing.TotalMs} ms.";

        if (timing.DbMs > 0)
            return $" Detect {timing.DetectMs} ms | DB {timing.DbMs} ms | Total {timing.TotalMs} ms.";

        return $" Detect {timing.DetectMs} ms | Total {timing.TotalMs} ms.";
    }

    private void ApplyTimingToStatus(bool includeRobotFetch)
    {
        string baseStatus = string.IsNullOrWhiteSpace(_pendingStatusWithoutTiming)
            ? TimingSuffixRegex.Replace(_status.Text, string.Empty).TrimEnd()
            : _pendingStatusWithoutTiming;
        _pendingStatusWithoutTiming = string.Empty;
        _status.Text = baseStatus + FormatTimingLine(_lastFrameTiming, includeRobotFetch);
    }

    private static Mat BuildDisplayMask(Mat roiMask, OpenCvSharp.Size frameSize, OpenCvSharp.Rect? roi)
    {
        if (!roi.HasValue)
            return roiMask.Clone();

        var fullMask = new Mat(frameSize, MatType.CV_8UC1, Scalar.Black);
        using var destinationRoi = new Mat(fullMask, roi.Value);

        if (roiMask.Size() == frameSize)
        {
            using var sourceRoi = new Mat(roiMask, roi.Value);
            sourceRoi.CopyTo(destinationRoi);
        }
        else
        {
            roiMask.CopyTo(destinationRoi);
        }

        return fullMask;
    }

    private Mat BuildDisplayFrame(Mat source, PictureBox target)
    {
        int targetWidth = _visionSettings.DisplayWidth;
        int targetHeight = _visionSettings.DisplayHeight;

        if (targetWidth <= 0 && targetHeight <= 0)
        {
            targetWidth = Math.Max(1, target.ClientSize.Width);
            targetHeight = Math.Max(1, target.ClientSize.Height);
        }

        if (targetWidth <= 0)
            targetWidth = Math.Max(1, (int)Math.Round(source.Width * (targetHeight / (double)source.Height)));

        if (targetHeight <= 0)
            targetHeight = Math.Max(1, (int)Math.Round(source.Height * (targetWidth / (double)source.Width)));

        var resized = new Mat();
        var interpolation = targetWidth < source.Width || targetHeight < source.Height
            ? InterpolationFlags.Area
            : InterpolationFlags.Nearest;
        Cv2.Resize(source, resized, new OpenCvSharp.Size(targetWidth, targetHeight), 0, 0, interpolation);
        return resized;
    }

    private OpenCvSharp.Rect? BuildDetectionRoi(Mat frame)
    {
        if (!_visionSettings.UseRoi) return null;

        int y = Math.Clamp(_visionSettings.RoiX, 0, Math.Max(0, frame.Height - 1));
        int x = Math.Clamp(_visionSettings.RoiY, 0, Math.Max(0, frame.Width - 1));
        int width = _visionSettings.RoiWidth <= 0 ? frame.Width - x : _visionSettings.RoiWidth;
        int height = _visionSettings.RoiHeight <= 0 ? frame.Height - y : _visionSettings.RoiHeight;

        width = Math.Clamp(width, 1, frame.Width - x);
        height = Math.Clamp(height, 1, frame.Height - y);
        return new OpenCvSharp.Rect(x, y, width, height);
    }

    private static List<DetectionResult> OffsetDetections(IEnumerable<DetectionResult> detections, int offsetX, int offsetY)
    {
        return detections
            .Select((d, index) =>
            {
                var center = new Point2f(d.Center.X + offsetX, d.Center.Y + offsetY);
                var box = new OpenCvSharp.Rect(
                    d.BoundingBox.X + offsetX,
                    d.BoundingBox.Y + offsetY,
                    d.BoundingBox.Width,
                    d.BoundingBox.Height);
                var rotatedBox = new RotatedRect(
                    new Point2f(d.RotatedBox.Center.X + offsetX, d.RotatedBox.Center.Y + offsetY),
                    d.RotatedBox.Size,
                    d.RotatedBox.Angle);

                return new DetectionResult
                {
                    Id = index + 1,
                    Center = center,
                    PixelX = center.Y,
                    PixelY = center.X,
                    Area = d.Area,
                    Radius = d.Radius,
                    Circularity = d.Circularity,
                    Confidence = d.Confidence,
                    Angle = d.Angle,
                    BoundingBox = box,
                    RotatedBox = rotatedBox,
                    ObjectXAxis = d.ObjectXAxis,
                    ObjectYAxis = d.ObjectYAxis,
                    SimilarityScore = d.SimilarityScore
                };
            })
            .ToList();
    }

    private static List<DetectionResult> FilterDetectionsByRoi(IEnumerable<DetectionResult> detections, OpenCvSharp.Rect roi)
    {
        return detections
            .Where(d => roi.Contains(new CvPoint(
                (int)Math.Round(d.Center.X),
                (int)Math.Round(d.Center.Y))))
            .Select((d, index) => d.WithId(index + 1))
            .ToList();
    }

    private static List<DetectionResult> ProjectDetectionsToWorkFrame(IEnumerable<DetectionResult> detections, int imageHeight)
    {
        return detections
            .Select((d, index) =>
            {
                double pixelX = imageHeight - d.Center.Y;
                double pixelY = d.Center.X;

                return new DetectionResult
                {
                    Id = index + 1,
                    Center = d.Center,
                    PixelX = pixelX,
                    PixelY = pixelY,
                    Area = d.Area,
                    Radius = d.Radius,
                    Circularity = d.Circularity,
                    Confidence = d.Confidence,
                    Angle = RobotAngleConverter.FromVisionClockwiseAngle(d.Angle),
                    BoundingBox = d.BoundingBox,
                    RotatedBox = d.RotatedBox,
                    ObjectXAxis = d.ObjectXAxis,
                    ObjectYAxis = d.ObjectYAxis,
                    SimilarityScore = d.SimilarityScore,
                    MaskContour = d.MaskContour
                };
            })
            .ToList();
    }

    private string PushDetectionsToIntegrations(IReadOnlyList<DetectionResult> detections)
    {
        if (_integrationWritesBlocked)
            return string.Empty;

        EnsureIntegrationClients();
        int flaskQueued = 0;
        int dbQueued = 0;
        int dbWriteFailures = 0;
        bool useFlask = _robotSettings.EnableFlaskApi && _flaskClient is not null && _flaskConnected;
        bool useLocalDb = _robotSettings.EnableLocalDatabase && _database is not null;

        int clearedPending = ClearPendingRobotQueue(useFlask, useLocalDb);

        if (detections.Count == 0)
            return clearedPending > 0
                ? $"Latest snapshot — detected 0 object(s), cleared {clearedPending} old robot row(s)."
                : "";

        foreach (var detection in detections)
        {
            var result = CoordinateMapper.ToVisionResult(detection, _visionSettings);
            if (useFlask && _flaskClient is not null)
            {
                bool visionOk = _flaskClient.SendVisionResult(result).GetAwaiter().GetResult();
                bool robotOk = _flaskClient.SendRobotCoordinates(result.X, result.Y, result.Angle, result.Name).GetAwaiter().GetResult();
                if (visionOk && robotOk)
                    flaskQueued++;
            }

            if (useLocalDb && _database is not null)
            {
                bool visionOk = _database.SaveVisionResult(result);
                bool robotOk = _database.SaveRobotCoordinate(result, sentToRobot: false);
                if (visionOk && robotOk)
                    dbQueued++;
                else
                    dbWriteFailures++;
            }
        }

        var parts = new List<string>();
        if (flaskQueued > 0)
            parts.Add($"Flask API: {flaskQueued}");
        if (dbQueued > 0)
            parts.Add($"local DB: {dbQueued} ({_database!.DatabasePath})");
        if (dbWriteFailures > 0)
        {
            string detail = _database?.LastError ?? "database is locked or unavailable";
            parts.Add($"local DB FAILED x{dbWriteFailures} ({detail}; close DB Browser and retry)");
        }

        if (parts.Count > 0)
        {
            string clearedText = clearedPending > 0 ? $" (cleared {clearedPending} old robot row(s))" : "";
            return "Latest snapshot — " + string.Join("; ", parts) + clearedText + ".";
        }

        if (_robotSettings.EnableFlaskApi && !useFlask)
            return "Flask enabled but OFF/ERR — enable local SQLite or fix Flask URL.";

        return "No robot/API/DB queue target enabled.";
    }

    private int ClearPendingRobotQueue(bool useFlask, bool useLocalDb)
    {
        int cleared = 0;
        if (useFlask && _flaskClient is not null)
            cleared = Math.Max(cleared, _flaskClient.ClearPendingRobotCoordinatesAsync().GetAwaiter().GetResult());

        if (useLocalDb && _database is not null)
            cleared = Math.Max(cleared, _database.ClearPendingRobotCoordinates());

        return cleared;
    }

    private void EnsureIntegrationClients()
    {
        if (_robotSettings.EnableFlaskApi && _flaskClient is null)
        {
            _flaskClient = new FlaskApiClient(_robotSettings.FlaskApiUrl, _robotSettings.FlaskTimeoutMs);
            _flaskClient.OnStatusChanged += (_, status) => BeginInvoke(new Action(() =>
            {
                _flaskConnected = status.StartsWith("OK:", StringComparison.OrdinalIgnoreCase);
            }));
            _ = CheckFlaskHealthAsync();
        }

        if (_robotSettings.EnableLocalDatabase && _database is null)
        {
            try
            {
                _database = new VisionDatabase(_robotSettings.GetResolvedDatabasePath());
            }
            catch (Exception ex)
            {
                _status.Text = $"Database init error: {ex.Message}";
            }
        }
    }

    private void ResetIntegrationClients()
    {
        StopRobotIntegration();
        _flaskClient?.Dispose();
        _flaskClient = null;
        _database?.Dispose();
        _database = null;
        _flaskConnected = false;
        EnsureIntegrationClients();
    }

    private async Task CheckFlaskHealthAsync()
    {
        FlaskApiClient? client = _flaskClient;
        _flaskConnected = client is not null && await client.CheckHealth().ConfigureAwait(false);
    }

    private static void DrawDetectionFrame(Mat image, OpenCvSharp.Rect? roi)
    {
        var origin = new CvPoint(0, image.Height - 1);
        int axisLength = Math.Min(120, Math.Max(40, Math.Min(image.Width, image.Height) / 6));
        int yEnd = Math.Min(image.Width - 1, origin.X + axisLength);
        int xEnd = Math.Max(0, origin.Y - axisLength);

        if (roi.HasValue)
            Cv2.Rectangle(image, roi.Value, new Scalar(0, 255, 255), 2);

        Cv2.ArrowedLine(image, origin, new CvPoint(origin.X, xEnd), new Scalar(255, 80, 80), 3, tipLength: 0.18);
        Cv2.ArrowedLine(image, origin, new CvPoint(yEnd, origin.Y), new Scalar(0, 255, 255), 3, tipLength: 0.18);
        Cv2.Circle(image, origin, 5, new Scalar(0, 0, 255), -1);
        Cv2.PutText(image, "X", new CvPoint(8, Math.Max(24, xEnd - 8)), HersheyFonts.HersheySimplex, 0.8, new Scalar(255, 80, 80), 2);
        Cv2.PutText(image, "Y", new CvPoint(Math.Min(image.Width - 24, yEnd + 8), Math.Max(24, origin.Y - 8)), HersheyFonts.HersheySimplex, 0.8, new Scalar(0, 255, 255), 2);
        Cv2.PutText(image, "(X=0,Y=0)", new CvPoint(8, Math.Max(24, origin.Y - 34)), HersheyFonts.HersheySimplex, 0.55, new Scalar(255, 255, 255), 2);
    }

    private void DrawDetections(Mat image, IReadOnlyList<DetectionResult> detections)
    {
        int length = _visionSettings.CrosshairLength;
        int boxThickness = OverlayLineThickness(image);
        foreach (var detection in detections)
        {
            var result = CoordinateMapper.ToVisionResult(detection, _visionSettings);
            int x = (int)Math.Round(detection.Center.X);
            int y = (int)Math.Round(detection.Center.Y);
            var center = new CvPoint(x, y);
            var xColor = new Scalar(255, 80, 80);
            var yColor = new Scalar(0, 255, 255);
            var dotColor = new Scalar(0, 0, 255);

            Cv2.Rectangle(image, detection.BoundingBox, new Scalar(255, 0, 255), boxThickness);

            var boxPoints = detection.RotatedBox.Points()
                .Select(p => new CvPoint((int)Math.Round(p.X), (int)Math.Round(p.Y)))
                .ToArray();
            Cv2.Polylines(image, new[] { boxPoints }, true, new Scalar(0, 220, 0), boxThickness);

            var xAxisEnd = AxisEnd(center, detection.ObjectXAxis, length, image.Width, image.Height);
            var yAxisEnd = AxisEnd(center, detection.ObjectYAxis, length, image.Width, image.Height);
            Cv2.ArrowedLine(image, center, xAxisEnd, xColor, 2, tipLength: 0.18);
            Cv2.ArrowedLine(image, center, yAxisEnd, yColor, 2, tipLength: 0.18);
            Cv2.PutText(image, "X'", xAxisEnd, HersheyFonts.HersheySimplex, 0.55, xColor, 2);
            Cv2.PutText(image, "Y'", yAxisEnd, HersheyFonts.HersheySimplex, 0.55, yColor, 2);
            Cv2.Circle(image, center, 6, dotColor, -1);
            Cv2.Circle(image, center, Math.Max(8, (int)Math.Round(detection.Radius)), new Scalar(255, 255, 255), 1);

            string label = $"#{detection.Id} X={result.X:F1} Y={result.Y:F1} A={detection.Angle:F1}";
            Cv2.PutText(
                image,
                label,
                new CvPoint(x + 10, Math.Max(20, y - 10)),
                HersheyFonts.HersheySimplex,
                0.62,
                new Scalar(255, 255, 255),
                2);
        }

        Cv2.PutText(
            image,
            $"{detections.Count} object(s)",
            new CvPoint(12, 28),
            HersheyFonts.HersheySimplex,
            0.8,
            new Scalar(255, 255, 255),
            2);
    }

    private static int OverlayLineThickness(Mat image)
    {
        return Math.Max(2, (int)Math.Round(Math.Min(image.Width, image.Height) / 700.0));
    }

    private static string FormatBoltSizeRangeSummary(BoltSizeStats stats)
    {
        var inv = CultureInfo.InvariantCulture;
        string wMin = stats.WidthMinMm?.ToString("F2", inv) ?? "-";
        string wMax = stats.WidthMaxMm?.ToString("F2", inv) ?? "-";
        string lMin = stats.LengthMinMm?.ToString("F2", inv) ?? "-";
        string lMax = stats.LengthMaxMm?.ToString("F2", inv) ?? "-";
        string areaMin = stats.AreaMin?.ToString("F0", inv) ?? "-";
        string areaMax = stats.AreaMax?.ToString("F0", inv) ?? "-";
        return $" Width {wMin}-{wMax} mm, Length {lMin}-{lMax} mm, Area {areaMin}-{areaMax} px^2.";
    }

    private string BuildM8StatusText(int countAll, int countAfterFilter)
    {
        double wMin = _visionSettings.M8HeadDiameterMinMm;
        double wMax = _visionSettings.M8HeadDiameterMaxMm;
        double lMin = _visionSettings.M8LengthMinMm;
        double lMax = _visionSettings.M8LengthMaxMm;
        double aMin = _visionSettings.M8AreaMin;
        double aMax = _visionSettings.M8AreaMax;
        if (_visionSettings.EnableM8SizeFilter && countAll != countAfterFilter)
            return $" M8 filter: {countAll} -> {countAfterFilter} (width {wMin:F1}-{wMax:F1} mm, length {lMin:F1}-{lMax:F1} mm, area {aMin:F0}-{aMax:F0} px^2).";
        if (_visionSettings.EnableM8SizeFilter)
            return $" M8 filter ON (width {wMin:F1}-{wMax:F1} mm, length {lMin:F1}-{lMax:F1} mm, area {aMin:F0}-{aMax:F0} px^2).";
        return $" M8 calibration: Width/Length/Area columns + M8?; preview width {wMin:F1}-{wMax:F1} mm, length {lMin:F1}-{lMax:F1} mm, area {aMin:F0}-{aMax:F0} px^2.";
    }

    private string BuildRobotSafetyStatusText(int countBefore, int countAfter, int rejected)
    {
        if (!_visionSettings.EnableRobotSafetyFilter && !_visionSettings.EnableSceneProximityFilter)
            return " Robot safety filter OFF.";

        if (!_visionSettings.EnableRobotSafetyFilter)
        {
            string sceneRule = "scene mask B1 proximity (B3)";
            if (rejected > 0)
                return $" Scene proximity: {countBefore} -> {countAfter}, rejected {rejected} target(s) ({sceneRule}).";
            return $" Scene proximity ON ({sceneRule}).";
        }

        double minMm = _visionSettings.RobotSafetyM8MinCenterSpacingMm + _visionSettings.RobotSafetyM8CenterSpacingMarginMm;
        string rule = $"360° center>={minMm:F1}mm";
        if (_visionSettings.EnableSceneProximityFilter)
            rule += ", scene mask B1 proximity (B3)";

        if (rejected > 0)
            return $" Robot safety: {countBefore} -> {countAfter}, rejected {rejected} target(s) ({rule}).";

        return $" Robot safety ON ({rule}).";
    }

    private void RefreshSceneBinaryContext(Mat sceneBinary8u)
    {
        _lastSceneContext?.Dispose();
        _lastSceneContext = _visionSettings.EnableSceneProximityFilter
            ? SceneBinaryContext.TryCreate(sceneBinary8u)
            : null;
    }

    private List<DetectionResult> ApplyRobotSafetyFilter(
        IReadOnlyList<DetectionResult> allDetections,
        IReadOnlyList<DetectionResult> candidates,
        out int rejectedCount) =>
        RobotSafetyFilter.ApplyForRobotSend(allDetections, candidates, _visionSettings, _lastSceneContext, out rejectedCount);

    private void UpdateGrid(IReadOnlyList<DetectionResult> detections)
    {
        _grid.Rows.Clear();
        foreach (var detection in detections)
        {
            var result = CoordinateMapper.ToVisionResult(detection, _visionSettings);
            string widthText;
            string lengthText;
            string m8Text;
            if (BoltSizeFilter.TryMeasureBoltSizeMm(detection, _visionSettings, out double widthMm, out double lengthMm))
            {
                widthText = widthMm.ToString("F2", CultureInfo.InvariantCulture);
                lengthText = lengthMm.ToString("F2", CultureInfo.InvariantCulture);
                m8Text = BoltSizeFilter.IsWithinM8SizeRange(widthMm, lengthMm, detection.Area, _visionSettings) ? "yes" : "no";
            }
            else
            {
                widthText = "-";
                lengthText = "-";
                m8Text = "-";
            }

            bool sendRobot = _lastRobotSendKeys.Contains(BoltSizeFilter.RobotDetectionKey(detection));
            string robotText = BoltSizeFilter.FormatRobotSendText(sendRobot);

            string confidenceText = VisionSettings.ToConfidencePercent(detection.Confidence)
                .ToString("F1", CultureInfo.InvariantCulture);

            _grid.Rows.Add(
                detection.Id,
                widthText,
                lengthText,
                m8Text,
                robotText,
                result.X.ToString("F1", CultureInfo.InvariantCulture),
                result.Y.ToString("F1", CultureInfo.InvariantCulture),
                detection.Angle.ToString("F1", CultureInfo.InvariantCulture),
                detection.Area.ToString("F0", CultureInfo.InvariantCulture),
                confidenceText,
                detection.Circularity.ToString("F2", CultureInfo.InvariantCulture));
        }

    }

    private static CvPoint AxisEnd(CvPoint center, Point2f axis, int length, int imageWidth, int imageHeight)
    {
        int x = Math.Clamp((int)Math.Round(center.X + axis.X * length), 0, imageWidth - 1);
        int y = Math.Clamp((int)Math.Round(center.Y + axis.Y * length), 0, imageHeight - 1);
        return new CvPoint(x, y);
    }

    private static void ShowMat(PictureBox box, Mat mat)
    {
        var bitmap = BitmapConverter.ToBitmap(mat);
        var old = box.Image;
        box.Image = bitmap;
        old?.Dispose();
    }

    private static void ClearPicture(PictureBox box)
    {
        var old = box.Image;
        box.Image = null;
        old?.Dispose();
    }

    private void SaveCsv()
    {
        if (_lastAllDetections.Count == 0)
        {
            _status.Text = "No detections to save.";
            return;
        }

        string exportDir = SystemConfig.EXPORT_CSV_DIRECTORY;
        Directory.CreateDirectory(exportDir);
        string path = Path.Combine(exportDir, $"bolt_pixels_{DateTime.Now:yyyyMMdd_HHmmss}.csv");

        const char sep = ';';
        var inv = CultureInfo.InvariantCulture;
        var csv = new StringBuilder();
        csv.AppendLine($"sep={sep}");
        csv.AppendLine(string.Join(sep, new[]
        {
            "id",
            "width_mm",
            "length_mm",
            "m8_in_preview_range",
            "send_to_robot",
            "x_robot",
            "y_robot",
            "angle_degrees",
            "area",
            "radius",
            "circularity",
            "confidence_percent",
            "bounding_x",
            "bounding_y",
            "bounding_width",
            "bounding_height",
            "center_x_image",
            "center_y_image",
            "rotated_width",
            "rotated_height",
            "rotated_angle_cv",
            "object_axis_x_dx",
            "object_axis_x_dy",
            "object_axis_y_dx",
            "object_axis_y_dy",
            "similarity_score"
        }));

        foreach (var d in _lastAllDetections)
        {
            var result = CoordinateMapper.ToVisionResult(d, _visionSettings);
            string widthMm = BoltSizeFilter.TryMeasureBoltSizeMm(d, _visionSettings, out double w, out double len)
                ? w.ToString("F3", inv) : "";
            string lengthMm = BoltSizeFilter.TryMeasureBoltSizeMm(d, _visionSettings, out double w2, out double l2)
                ? l2.ToString("F3", inv) : "";
            string m8Preview = BoltSizeFilter.TryMeasureBoltSizeMm(d, _visionSettings, out double w3, out double l3) &&
                               BoltSizeFilter.IsWithinM8SizeRange(w3, l3, d.Area, _visionSettings)
                ? "yes"
                : BoltSizeFilter.TryMeasureBoltSizeMm(d, _visionSettings, out _, out _) ? "no" : "";
            string sendRobot = BoltSizeFilter.FormatRobotSendText(
                _lastRobotSendKeys.Contains(BoltSizeFilter.RobotDetectionKey(d)));
            csv.AppendLine(string.Join(sep, new[]
            {
                d.Id.ToString(inv),
                widthMm,
                lengthMm,
                m8Preview,
                sendRobot,
                result.X.ToString("F3", inv),
                result.Y.ToString("F3", inv),
                d.Angle.ToString("F3", inv),
                d.Area.ToString("F3", inv),
                d.Radius.ToString("F3", inv),
                d.Circularity.ToString("F5", inv),
                VisionSettings.ToConfidencePercent(d.Confidence).ToString("F1", inv),
                d.BoundingBox.X.ToString(inv),
                d.BoundingBox.Y.ToString(inv),
                d.BoundingBox.Width.ToString(inv),
                d.BoundingBox.Height.ToString(inv),
                d.Center.X.ToString("F3", inv),
                d.Center.Y.ToString("F3", inv),
                d.RotatedBox.Size.Width.ToString("F3", inv),
                d.RotatedBox.Size.Height.ToString("F3", inv),
                d.RotatedBox.Angle.ToString("F3", inv),
                d.ObjectXAxis.X.ToString("F5", inv),
                d.ObjectXAxis.Y.ToString("F5", inv),
                d.ObjectYAxis.X.ToString("F5", inv),
                d.ObjectYAxis.Y.ToString("F5", inv),
                d.SimilarityScore.ToString("F5", inv)
            }));
        }

        File.WriteAllText(path, csv.ToString(), new UTF8Encoding(encoderShouldEmitUTF8Identifier: true));
        _status.Text = $"Saved CSV: {Path.GetFullPath(path)}";
    }

    private void SaveImage()
    {
        if (_neptune is null || !_neptune.IsOpened)
        {
            _status.Text = "Start the system first, then press Save image.";
            return;
        }

        if (_isProcessingFrame)
        {
            _status.Text = "Capture already in progress.";
            return;
        }

        _isProcessingFrame = true;
        try
        {
            var grabbed = GrabFreshFrame(_neptune);
            if (grabbed is null || grabbed.Empty())
            {
                grabbed?.Dispose();
                _status.Text = "No frame from camera.";
                return;
            }

            int snapshotIndex = ++_snapshotIndex;
            using (grabbed)
            {
                using var frame = PrepareCameraFrame(grabbed);
                ProcessFrame(frame, autoSave: true, snapshotIndex: snapshotIndex);
            }
        }
        catch (Exception ex)
        {
            _status.Text = $"Save image error: {ex.Message}";
        }
        finally
        {
            _isProcessingFrame = false;
        }
    }

    private string SaveAnnotatedImage()
    {
        if (_lastAnnotatedFrame is null)
            throw new InvalidOperationException("No annotated frame to save.");

        string exportDir = SystemConfig.EXPORT_IMAGES_DIRECTORY;
        Directory.CreateDirectory(exportDir);
        string path = Path.Combine(exportDir, $"bolt_pixels_{DateTime.Now:yyyyMMdd_HHmmss}.png");
        _lastAnnotatedFrame.SaveImage(path);
        return path;
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            StopCamera();
            _lastAnnotatedFrame?.Dispose();
            _flaskClient?.Dispose();
            _database?.Dispose();
            _yoloDetector.Dispose();
            _timer.Dispose();
            _toolTip.Dispose();
            ClearPicture(_cameraView);
            ClearPicture(_maskView);
        }

        base.Dispose(disposing);
    }
}
