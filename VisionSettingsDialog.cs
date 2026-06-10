namespace BoltPixelDetectorApp;

public sealed class VisionSettingsDialog : Form
{
    private const int RowHeight = 36;
    private const int FooterHeight = 82;
    private const int FooterButtonBottomMargin = 10;
    private const int BrowseColumnWidth = 54;
    private const int ScrollGridBottomPadding = 32;
    private const int LabelColumnPercent = 44;
    private const int CameraRows = 4;
    private const int DetectionRows = 29;

    private readonly VisionSettings _settings;

    private readonly TextBox _sdkRoot = new();
    private readonly NumericUpDown _exposure = new();
    private readonly ComboBox _detectorMode = new();
    private readonly TextBox _yoloModelPath = new();
    private readonly MaterialButton _browseYoloModel = new();
    private readonly TextBox _yoloPythonExe = new();
    private readonly NumericUpDown _yoloImageSize = new();
    private readonly NumericUpDown _yoloNms = new();
    private readonly NumericUpDown _displayWidth = new();
    private readonly NumericUpDown _displayHeight = new();
    private readonly SquareToggle _useRoi = new();
    private readonly NumericUpDown _roiX = new();
    private readonly NumericUpDown _roiY = new();
    private readonly NumericUpDown _roiWidth = new();
    private readonly NumericUpDown _roiHeight = new();
    private readonly NumericUpDown _threshold = new();
    private readonly NumericUpDown _minArea = new();
    private readonly NumericUpDown _maxArea = new();
    private readonly NumericUpDown _minCircularity = new();
    private readonly NumericUpDown _minConfidence = new();
    private readonly NumericUpDown _crosshairLength = new();
    private readonly SquareToggle _invert = new();
    private readonly SquareToggle _showMask = new();
    private readonly SquareToggle _enableM8Filter = new();
    private readonly NumericUpDown _m8HeadMinMm = new();
    private readonly NumericUpDown _m8HeadMaxMm = new();
    private readonly NumericUpDown _m8LengthMinMm = new();
    private readonly NumericUpDown _m8LengthMaxMm = new();
    private readonly NumericUpDown _m8AreaMin = new();
    private readonly NumericUpDown _m8AreaMax = new();
    private readonly SquareToggle _enableRobotSafetyFilter = new();
    private readonly NumericUpDown _robotM8MinCenterSpacingMm = new();
    private readonly NumericUpDown _robotM8CenterSpacingMarginMm = new();
    private readonly NumericUpDown _robotM8MinCenterSpacingPx = new();

    public VisionSettingsDialog(VisionSettings settings)
    {
        _settings = settings;
        Text = "Vision settings";
        MinimumSize = new Size(640, 720);
        Size = new Size(680, 880);
        StartPosition = FormStartPosition.CenterParent;
        FormBorderStyle = FormBorderStyle.Sizable;
        MaximizeBox = true;
        MinimizeBox = true;
        UiTheme.ApplyToForm(this);

        BuildLayout();
        ConfigureInputs();
        LoadSettings();
    }

    private void BuildLayout()
    {
        int cameraBlockHeight = MeasureCardContentHeight(CameraRows);

        var shell = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = Padding.Empty,
            BackColor = UiTheme.Surface
        };
        shell.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        shell.RowStyles.Add(new RowStyle(SizeType.Absolute, FooterHeight));

        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = new Padding(UiTheme.SpacingLg, UiTheme.SpacingLg, UiTheme.SpacingLg, UiTheme.SpacingSm),
            BackColor = UiTheme.Surface
        };
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, cameraBlockHeight));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        var footer = new Panel
        {
            Dock = DockStyle.Fill,
            Padding = new Padding(
                UiTheme.SpacingLg,
                UiTheme.SpacingSm,
                UiTheme.SpacingLg + 8,
                UiTheme.SpacingMd + FooterButtonBottomMargin),
            BackColor = UiTheme.Surface
        };
        var cancel = new MaterialButton { Text = "Cancel", DialogResult = DialogResult.Cancel, Width = 92, Height = 34 };
        var ok = new MaterialButton { Text = "Save", DialogResult = DialogResult.OK, Width = 92, Height = 34 };
        cancel.Variant = MaterialButtonVariant.Outlined;
        ok.Variant = MaterialButtonVariant.Filled;
        ok.Click += (_, _) => SaveSettings();
        cancel.Anchor = AnchorStyles.Right | AnchorStyles.Top;
        ok.Anchor = AnchorStyles.Right | AnchorStyles.Top;
        footer.Controls.Add(cancel);
        footer.Controls.Add(ok);

        void LayoutFooterButtons()
        {
            var area = footer.DisplayRectangle;
            if (area.Width <= 0 || area.Height <= 0)
                return;
            int y = area.Bottom - cancel.Height;
            ok.Location = new Point(area.Right - ok.Width, y);
            cancel.Location = new Point(Math.Max(area.Left, ok.Left - UiTheme.SpacingSm - cancel.Width), y);
        }

        footer.Resize += (_, _) => LayoutFooterButtons();
        Shown += (_, _) => LayoutFooterButtons();

        shell.Controls.Add(root, 0, 0);
        shell.Controls.Add(footer, 0, 1);
        Controls.Add(shell);

        AcceptButton = ok;
        CancelButton = cancel;

        var cameraGroup = new CardPanel { Title = "Camera / display", Dock = DockStyle.Fill };
        var cameraGrid = CreateGrid(CameraRows);
        cameraGrid.Padding = new Padding(6, 4, 6, UiTheme.SpacingMd);
        cameraGrid.Dock = DockStyle.Top;
        cameraGroup.Controls.Add(cameraGrid);
        root.Controls.Add(cameraGroup, 0, 0);

        AddRow(cameraGrid, 0, "SDK root", _sdkRoot);
        AddRow(cameraGrid, 1, "Exposure us", _exposure);
        AddRow(cameraGrid, 2, "Display width (0=orig)", _displayWidth);
        AddRow(cameraGrid, 3, "Display height (0=orig)", _displayHeight);

        var detectionGroup = new CardPanel { Title = "Detection / ROI", Dock = DockStyle.Fill };
        var detectionScroll = CreateScrollPanel();
        var detectionGrid = CreateGrid(DetectionRows);
        detectionGrid.Padding = new Padding(6, 4, 6, ScrollGridBottomPadding);
        detectionScroll.Controls.Add(detectionGrid);
        detectionGroup.Controls.Add(detectionScroll);
        root.Controls.Add(detectionGroup, 0, 1);

        _detectorMode.DropDownStyle = ComboBoxStyle.DropDownList;
        _detectorMode.Items.AddRange(new object[] { "OpenCV", "YOLOv8 Segmentation", "OpenCV + YOLO (fused mask)" });

        _yoloModelPath.ReadOnly = true;
        _yoloModelPath.Cursor = Cursors.Hand;
        _yoloModelPath.Click += (_, _) => BrowseYoloModel();

        _browseYoloModel.Text = "Browse";
        _browseYoloModel.Click += (_, _) => BrowseYoloModel();
        StyleBrowseButton(_browseYoloModel);

        var modelPicker = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            Margin = Padding.Empty,
            BackColor = UiTheme.SurfaceContainer
        };
        modelPicker.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        modelPicker.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, BrowseColumnWidth));
        var yoloPathHost = MaterialInputHost.Wrap(_yoloModelPath);
        yoloPathHost.Dock = DockStyle.Fill;
        yoloPathHost.Margin = new Padding(0, 2, 2, 2);
        _browseYoloModel.Dock = DockStyle.Fill;
        _browseYoloModel.Margin = new Padding(0, 2, 0, 2);
        modelPicker.Controls.Add(yoloPathHost, 0, 0);
        modelPicker.Controls.Add(_browseYoloModel, 1, 0);

        AddRow(detectionGrid, 0, "Detector mode", _detectorMode);
        AddRow(detectionGrid, 1, "YOLO model (.onnx/.pt)", modelPicker);
        AddRow(detectionGrid, 2, "Python exe for .pt", _yoloPythonExe);
        AddRow(detectionGrid, 3, "YOLO image size", _yoloImageSize);
        AddRow(detectionGrid, 4, "YOLO NMS", _yoloNms);
        AddRow(detectionGrid, 5, "Use ROI", _useRoi);
        AddRow(detectionGrid, 6, "ROI X (maps to image Y)", _roiX);
        AddRow(detectionGrid, 7, "ROI Y (maps to image X)", _roiY);
        AddRow(detectionGrid, 8, "ROI width (0=rest)", _roiWidth);
        AddRow(detectionGrid, 9, "ROI height (0=rest)", _roiHeight);
        AddRow(detectionGrid, 10, "Threshold (0=Otsu)", _threshold);
        AddRow(detectionGrid, 11, "Min area", _minArea);
        AddRow(detectionGrid, 12, "Max area", _maxArea);
        AddRow(detectionGrid, 13, "Min circularity", _minCircularity);
        AddRow(detectionGrid, 14, "Min confidence filter (%)", _minConfidence);
        AddRow(detectionGrid, 15, "Crosshair length", _crosshairLength);
        AddRow(detectionGrid, 16, "Invert binary", _invert);
        AddRow(detectionGrid, 17, "Show mask", _showMask);
        AddRow(detectionGrid, 18, "M8 size filter (mm)", _enableM8Filter);
        AddRow(detectionGrid, 19, "M8 width min (mm)", _m8HeadMinMm);
        AddRow(detectionGrid, 20, "M8 width max (mm)", _m8HeadMaxMm);
        AddRow(detectionGrid, 21, "M8 length min (mm)", _m8LengthMinMm);
        AddRow(detectionGrid, 22, "M8 length max (mm)", _m8LengthMaxMm);
        AddRow(detectionGrid, 23, "M8 area min (px^2)", _m8AreaMin);
        AddRow(detectionGrid, 24, "M8 area max (px^2)", _m8AreaMax);
        AddRow(detectionGrid, 25, "Robot safety filter (360° center)", _enableRobotSafetyFilter);
        AddRow(detectionGrid, 26, "Min center spacing (mm)", _robotM8MinCenterSpacingMm);
        AddRow(detectionGrid, 27, "Center spacing margin (mm)", _robotM8CenterSpacingMarginMm);
        AddRow(detectionGrid, 28, "Min center spacing (px, 0=auto)", _robotM8MinCenterSpacingPx);

        ApplyScrollGridHeight(detectionGrid, DetectionRows);
        detectionScroll.Resize += (_, _) => SyncGridWidth(detectionScroll, detectionGrid);
        detectionGrid.PerformLayout();
    }

    private static int MeasureCardContentHeight(int rowCount)
    {
        // Title band + card padding + grid padding + rows + bottom breathing room.
        int titleBand = 26 + UiTheme.SpacingSm;
        int gridPadding = 12;
        int bottom = UiTheme.SpacingMd + UiTheme.SpacingSm;
        return titleBand + gridPadding + rowCount * RowHeight + bottom;
    }

    private static Panel CreateScrollPanel()
    {
        var panel = new Panel
        {
            Dock = DockStyle.Fill,
            AutoScroll = true,
            AutoScrollMargin = new Size(0, UiTheme.SpacingLg),
            BackColor = UiTheme.SurfaceContainer
        };
        EnableDoubleBuffer(panel);
        return panel;
    }

    private static void ApplyScrollGridHeight(TableLayoutPanel grid, int rowCount)
    {
        int height = grid.Padding.Vertical + rowCount * RowHeight + UiTheme.SpacingSm;
        grid.AutoSize = false;
        grid.AutoSizeMode = AutoSizeMode.GrowOnly;
        grid.Height = height;
        grid.MinimumSize = new Size(0, height);
    }

    private static void EnableDoubleBuffer(Control control)
    {
        typeof(Control).InvokeMember(
            "DoubleBuffered",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.SetProperty,
            null,
            control,
            new object[] { true });
    }

    private static void SyncGridWidth(Control scrollHost, TableLayoutPanel grid)
    {
        int width = Math.Max(320, scrollHost.ClientSize.Width - SystemInformation.VerticalScrollBarWidth - 8);
        if (grid.Width != width)
            grid.Width = width;
    }

    private static TableLayoutPanel CreateGrid(int rowCount)
    {
        var grid = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            ColumnCount = 2,
            RowCount = rowCount,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            BackColor = UiTheme.SurfaceContainer,
            Padding = new Padding(6, 4, 6, 8)
        };
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, LabelColumnPercent));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100 - LabelColumnPercent));
        for (int i = 0; i < rowCount; i++)
            grid.RowStyles.Add(new RowStyle(SizeType.Absolute, RowHeight));
        return grid;
    }

    private void ConfigureInputs()
    {
        _exposure.Minimum = SystemConfig.NEPTUNE_EXPOSURE_MIN;
        _exposure.Maximum = SystemConfig.NEPTUNE_EXPOSURE_MAX;
        _exposure.Increment = 500;
        ConfigureNumber(_displayWidth, 0, 10000, 10);
        ConfigureNumber(_displayHeight, 0, 10000, 10);
        ConfigureNumber(_yoloImageSize, 320, 2048, 32);
        ConfigureDecimal(_yoloNms, 0, 1, 2, 0.05M);
        ConfigureNumber(_roiX, 0, 10000, 10);
        ConfigureNumber(_roiY, 0, 10000, 10);
        ConfigureNumber(_roiWidth, 0, 10000, 10);
        ConfigureNumber(_roiHeight, 0, 10000, 10);
        ConfigureNumber(_threshold, 0, 255, 1);
        ConfigureDecimal(_minArea, 1, 1000000, 0, 100);
        ConfigureDecimal(_maxArea, 1, 5000000, 0, 500);
        ConfigureDecimal(_minCircularity, 0, 1, 2, 0.05M);
        ConfigureDecimal(_minConfidence, 0, 100, 0, 5);
        ConfigureNumber(_crosshairLength, 10, 500, 5);
        ConfigureDecimal(_m8HeadMinMm, 5, 100, 1, 0.5M);
        ConfigureDecimal(_m8HeadMaxMm, 5, 100, 1, 0.5M);
        ConfigureDecimal(_m8LengthMinMm, 5, 100, 1, 0.5M);
        ConfigureDecimal(_m8LengthMaxMm, 5, 100, 1, 0.5M);
        ConfigureDecimal(_m8AreaMin, 0, 5000000, 0, 500);
        ConfigureDecimal(_m8AreaMax, 0, 5000000, 0, 500);
        ConfigureDecimal(_robotM8MinCenterSpacingMm, 0, 200, 1, 0.5M);
        ConfigureDecimal(_robotM8CenterSpacingMarginMm, 0, 50, 1, 0.5M);
        ConfigureDecimal(_robotM8MinCenterSpacingPx, 0, 5000, 0, 5);
    }

    private void LoadSettings()
    {
        _sdkRoot.Text = _settings.SdkRoot;
        _exposure.Value = ClampDecimal(_settings.ExposureUs, _exposure.Minimum, _exposure.Maximum);
        _detectorMode.SelectedIndex = Math.Clamp(_settings.DetectorMode, 0, 2);
        _yoloModelPath.Text = _settings.YoloModelPath;
        _yoloPythonExe.Text = _settings.YoloPythonExe;
        _yoloImageSize.Value = ClampDecimal(_settings.YoloImageSize, _yoloImageSize.Minimum, _yoloImageSize.Maximum);
        _yoloNms.Value = ClampDecimal((decimal)_settings.YoloNmsThreshold, _yoloNms.Minimum, _yoloNms.Maximum);
        _displayWidth.Value = ClampDecimal(_settings.DisplayWidth, _displayWidth.Minimum, _displayWidth.Maximum);
        _displayHeight.Value = ClampDecimal(_settings.DisplayHeight, _displayHeight.Minimum, _displayHeight.Maximum);
        _useRoi.Checked = _settings.UseRoi;
        _roiX.Value = ClampDecimal(_settings.RoiX, _roiX.Minimum, _roiX.Maximum);
        _roiY.Value = ClampDecimal(_settings.RoiY, _roiY.Minimum, _roiY.Maximum);
        _roiWidth.Value = ClampDecimal(_settings.RoiWidth, _roiWidth.Minimum, _roiWidth.Maximum);
        _roiHeight.Value = ClampDecimal(_settings.RoiHeight, _roiHeight.Minimum, _roiHeight.Maximum);
        _threshold.Value = ClampDecimal(_settings.Threshold, _threshold.Minimum, _threshold.Maximum);
        _minArea.Value = ClampDecimal((decimal)_settings.MinArea, _minArea.Minimum, _minArea.Maximum);
        _maxArea.Value = ClampDecimal((decimal)_settings.MaxArea, _maxArea.Minimum, _maxArea.Maximum);
        _minCircularity.Value = ClampDecimal((decimal)_settings.MinCircularity, _minCircularity.Minimum, _minCircularity.Maximum);
        _minConfidence.Value = ClampDecimal((decimal)_settings.MinConfidencePercent, _minConfidence.Minimum, _minConfidence.Maximum);
        _crosshairLength.Value = ClampDecimal(_settings.CrosshairLength, _crosshairLength.Minimum, _crosshairLength.Maximum);
        _invert.Checked = _settings.InvertBinary;
        _showMask.Checked = _settings.ShowMask;
        _enableM8Filter.Checked = _settings.EnableM8SizeFilter;
        _m8HeadMinMm.Value = ClampDecimal((decimal)_settings.M8HeadDiameterMinMm, _m8HeadMinMm.Minimum, _m8HeadMinMm.Maximum);
        _m8HeadMaxMm.Value = ClampDecimal((decimal)_settings.M8HeadDiameterMaxMm, _m8HeadMaxMm.Minimum, _m8HeadMaxMm.Maximum);
        _m8LengthMinMm.Value = ClampDecimal((decimal)_settings.M8LengthMinMm, _m8LengthMinMm.Minimum, _m8LengthMaxMm.Maximum);
        _m8LengthMaxMm.Value = ClampDecimal((decimal)_settings.M8LengthMaxMm, _m8LengthMinMm.Minimum, _m8LengthMaxMm.Maximum);
        _m8AreaMin.Value = ClampDecimal((decimal)_settings.M8AreaMin, _m8AreaMin.Minimum, _m8AreaMax.Maximum);
        _m8AreaMax.Value = ClampDecimal((decimal)_settings.M8AreaMax, _m8AreaMin.Minimum, _m8AreaMax.Maximum);
        _enableRobotSafetyFilter.Checked = _settings.EnableRobotSafetyFilter;
        _robotM8MinCenterSpacingMm.Value = ClampDecimal((decimal)_settings.RobotSafetyM8MinCenterSpacingMm, _robotM8MinCenterSpacingMm.Minimum, _robotM8MinCenterSpacingMm.Maximum);
        _robotM8CenterSpacingMarginMm.Value = ClampDecimal((decimal)_settings.RobotSafetyM8CenterSpacingMarginMm, _robotM8CenterSpacingMarginMm.Minimum, _robotM8CenterSpacingMarginMm.Maximum);
        _robotM8MinCenterSpacingPx.Value = ClampDecimal((decimal)_settings.RobotSafetyM8MinCenterSpacingPx, _robotM8MinCenterSpacingPx.Minimum, _robotM8MinCenterSpacingPx.Maximum);
    }

    private void SaveSettings()
    {
        _settings.SdkRoot = _sdkRoot.Text.Trim();
        _settings.ExposureUs = (int)_exposure.Value;
        _settings.DetectorMode = _detectorMode.SelectedIndex;
        _settings.YoloModelPath = _yoloModelPath.Text.Trim();
        _settings.YoloPythonExe = _yoloPythonExe.Text.Trim();
        _settings.YoloImageSize = (int)_yoloImageSize.Value;
        _settings.YoloNmsThreshold = (double)_yoloNms.Value;
        _settings.DisplayWidth = (int)_displayWidth.Value;
        _settings.DisplayHeight = (int)_displayHeight.Value;
        _settings.UseRoi = _useRoi.Checked;
        _settings.RoiX = (int)_roiX.Value;
        _settings.RoiY = (int)_roiY.Value;
        _settings.RoiWidth = (int)_roiWidth.Value;
        _settings.RoiHeight = (int)_roiHeight.Value;
        _settings.Threshold = (int)_threshold.Value;
        _settings.MinArea = (double)_minArea.Value;
        _settings.MaxArea = (double)_maxArea.Value;
        _settings.MinCircularity = (double)_minCircularity.Value;
        _settings.MinConfidencePercent = (double)_minConfidence.Value;
        _settings.CrosshairLength = (int)_crosshairLength.Value;
        _settings.InvertBinary = _invert.Checked;
        _settings.ShowMask = _showMask.Checked;
        _settings.EnableM8SizeFilter = _enableM8Filter.Checked;
        _settings.M8HeadDiameterMinMm = (double)_m8HeadMinMm.Value;
        _settings.M8HeadDiameterMaxMm = (double)_m8HeadMaxMm.Value;
        _settings.M8LengthMinMm = (double)_m8LengthMinMm.Value;
        _settings.M8LengthMaxMm = (double)_m8LengthMaxMm.Value;
        _settings.M8AreaMin = (double)_m8AreaMin.Value;
        _settings.M8AreaMax = (double)_m8AreaMax.Value;
        _settings.EnableRobotSafetyFilter = _enableRobotSafetyFilter.Checked;
        _settings.RobotSafetyM8MinCenterSpacingMm = (double)_robotM8MinCenterSpacingMm.Value;
        _settings.RobotSafetyM8CenterSpacingMarginMm = (double)_robotM8CenterSpacingMarginMm.Value;
        _settings.RobotSafetyM8MinCenterSpacingPx = (double)_robotM8MinCenterSpacingPx.Value;
        _settings.Save();
        _settings.ApplyToSystemConfig();
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

        string? currentDirectory = Path.GetDirectoryName(_yoloModelPath.Text.Trim());
        if (!string.IsNullOrWhiteSpace(currentDirectory) && Directory.Exists(currentDirectory))
            dialog.InitialDirectory = currentDirectory;
        else if (Directory.Exists(AppContext.BaseDirectory))
            dialog.InitialDirectory = AppContext.BaseDirectory;

        if (dialog.ShowDialog(this) == DialogResult.OK)
            _yoloModelPath.Text = dialog.FileName;
    }

    private static void AddRow(TableLayoutPanel panel, int row, string label, Control control)
    {
        var text = new Label
        {
            Text = label,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            AutoEllipsis = true,
            ForeColor = UiTheme.OnSurfaceVariant,
            Font = UiTheme.UiFontSmall,
            BackColor = UiTheme.SurfaceContainer,
            Padding = new Padding(0, 0, UiTheme.SpacingSm, 0)
        };

        Control field = control switch
        {
            SquareToggle toggle => CreateToggleCell(toggle),
            TableLayoutPanel layout => layout,
            _ => MaterialInputHost.Wrap(control)
        };
        field.Dock = DockStyle.Fill;
        panel.Controls.Add(text, 0, row);
        panel.Controls.Add(field, 1, row);
    }

    private static Control CreateToggleCell(SquareToggle toggle)
    {
        var cell = new Panel
        {
            Dock = DockStyle.Fill,
            BackColor = UiTheme.SurfaceContainer,
            Margin = Padding.Empty
        };
        toggle.BackColor = UiTheme.SurfaceContainer;
        cell.Controls.Add(toggle);
        cell.Resize += (_, _) =>
        {
            toggle.Location = new Point(
                2,
                Math.Max(0, (cell.ClientSize.Height - toggle.Height) / 2));
        };
        return cell;
    }

    private static void StyleBrowseButton(MaterialButton button)
    {
        button.Dock = DockStyle.Fill;
        button.Variant = MaterialButtonVariant.Outlined;
        button.Font = new Font(UiTheme.UiFontSmall.FontFamily, 8.25f);
        button.Height = 28;
        button.MinimumSize = new Size(48, 26);
        button.MaximumSize = new Size(BrowseColumnWidth + 2, 30);
        button.Padding = new Padding(4, 2, 4, 2);
        button.TabStop = false;
        button.Text = "Browse";
    }

    private static void ConfigureNumber(NumericUpDown input, decimal min, decimal max, decimal increment)
    {
        input.Minimum = min;
        input.Maximum = max;
        input.Increment = increment;
    }

    private static void ConfigureDecimal(NumericUpDown input, decimal min, decimal max, int decimalPlaces, decimal increment)
    {
        input.Minimum = min;
        input.Maximum = max;
        input.DecimalPlaces = decimalPlaces;
        input.Increment = increment;
    }

    private static decimal ClampDecimal(decimal value, decimal min, decimal max)
    {
        return Math.Min(max, Math.Max(min, value));
    }
}
