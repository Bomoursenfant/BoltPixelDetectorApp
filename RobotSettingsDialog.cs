namespace BoltPixelDetectorApp;

public sealed class RobotSettingsDialog : Form
{
    private const int RowHeight = 36;
    private const int FooterHeight = 82;
    private const int FooterButtonBottomMargin = 10;
    private const int BrowseColumnWidth = 54;
    private const int ScrollGridBottomPadding = 32;
    private const int LabelColumnPercent = 44;
    private const int RobotRows = 13;
    private const int IntegrationRows = 2;

    private readonly RobotConnectionSettings _settings;

    private readonly ComboBox _mode = new();
    private readonly TextBox _robotIp = new();
    private readonly NumericUpDown _robotPort = new();
    private readonly TextBox _pcListenIp = new();
    private readonly NumericUpDown _pcListenPort = new();
    private readonly ComboBox _protocol = new();
    private readonly SquareToggle _restrictIp = new();
    private readonly SquareToggle _bigEndian = new();
    private readonly NumericUpDown _maxBatch = new();
    private readonly NumericUpDown _testX = new();
    private readonly NumericUpDown _testY = new();
    private readonly NumericUpDown _testAngle = new();
    private readonly SquareToggle _serverTest = new();
    private readonly SquareToggle _enableFlask = new();
    private readonly TextBox _flaskUrl = new();
    private readonly SquareToggle _enableDb = new();
    private readonly TextBox _databasePath = new();
    private readonly MaterialButton _browseDb = new();

    public RobotSettingsDialog(RobotConnectionSettings settings)
    {
        _settings = settings;
        Text = "Robot & integrations";
        MinimumSize = new Size(640, 720);
        Size = new Size(660, 900);
        StartPosition = FormStartPosition.CenterParent;
        FormBorderStyle = FormBorderStyle.Sizable;
        MaximizeBox = true;
        MinimizeBox = true;
        UiTheme.ApplyToForm(this);

        BuildLayout();
        LoadSettings();
    }

    private void BuildLayout()
    {
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
            RowCount = 3,
            Padding = new Padding(UiTheme.SpacingLg, UiTheme.SpacingLg, UiTheme.SpacingLg, UiTheme.SpacingSm),
            BackColor = UiTheme.Surface
        };
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, MeasureCardContentHeight(IntegrationRows)));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, MeasureCardContentHeight(IntegrationRows)));

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

        var robotGroup = new CardPanel { Title = "Robot socket", Dock = DockStyle.Fill };
        var robotScroll = CreateScrollPanel();
        var robot = CreateGrid(RobotRows);
        robot.Padding = new Padding(6, 4, 6, ScrollGridBottomPadding);
        robotScroll.Controls.Add(robot);
        robotGroup.Controls.Add(robotScroll);
        root.Controls.Add(robotGroup, 0, 0);
        robotScroll.Resize += (_, _) => SyncGridWidth(robotScroll, robot);

        _mode.DropDownStyle = ComboBoxStyle.DropDownList;
        _mode.Items.AddRange(new object[] { "Server - PC listens for Robot/CFD", "Client - PC connects to Robot" });
        _protocol.DropDownStyle = ComboBoxStyle.DropDownList;
        _protocol.Items.AddRange(new object[] { "Nachi binary batch", "ASCII line" });
        AddRow(robot, 0, "Mode", _mode);
        AddRow(robot, 1, "Robot IP", _robotIp);
        AddRow(robot, 2, "Robot Port", _robotPort);
        AddRow(robot, 3, "PC Listen IP", _pcListenIp);
        AddRow(robot, 4, "PC Listen Port", _pcListenPort);
        AddRow(robot, 5, "Protocol", _protocol);
        AddRow(robot, 6, "Restrict to Robot IP", _restrictIp);
        AddRow(robot, 7, "Big-endian protocol", _bigEndian);
        AddRow(robot, 8, "Max batch", _maxBatch);
        AddRow(robot, 9, "Test X", _testX);
        AddRow(robot, 10, "Test Y", _testY);
        AddRow(robot, 11, "Test Angle", _testAngle);
        AddRow(robot, 12, "Use server test coordinates", _serverTest);
        ApplyScrollGridHeight(robot, RobotRows);

        var flaskGroup = new CardPanel { Title = "Flask API", Dock = DockStyle.Fill };
        var flask = CreateGrid(IntegrationRows);
        flask.Padding = new Padding(6, 4, 6, UiTheme.SpacingMd);
        flaskGroup.Controls.Add(flask);
        root.Controls.Add(flaskGroup, 0, 1);
        AddRow(flask, 0, "Enable Flask API", _enableFlask);
        AddRow(flask, 1, "API URL", _flaskUrl);

        var dbGroup = new CardPanel { Title = "SQLite database", Dock = DockStyle.Fill };
        var db = CreateGrid(IntegrationRows);
        db.Padding = new Padding(6, 4, 6, UiTheme.SpacingMd);
        dbGroup.Controls.Add(db);
        root.Controls.Add(dbGroup, 0, 2);
        AddRow(db, 0, "Enable local SQLite", _enableDb);

        _browseDb.Text = "Browse";
        _browseDb.Click += (_, _) => BrowseDatabasePath();
        StyleBrowseButton(_browseDb);

        var dbPicker = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            Margin = Padding.Empty,
            BackColor = UiTheme.SurfaceContainer
        };
        dbPicker.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        dbPicker.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, BrowseColumnWidth));
        var dbPathHost = MaterialInputHost.Wrap(_databasePath);
        dbPathHost.Dock = DockStyle.Fill;
        dbPathHost.Margin = new Padding(0, 2, 2, 2);
        _browseDb.Dock = DockStyle.Fill;
        _browseDb.Margin = new Padding(0, 2, 0, 2);
        dbPicker.Controls.Add(dbPathHost, 0, 0);
        dbPicker.Controls.Add(_browseDb, 1, 0);
        AddRow(db, 1, "DB path", dbPicker);

        AcceptButton = ok;
        CancelButton = cancel;
        ConfigureInputs();
    }

    private static int MeasureCardContentHeight(int rowCount)
    {
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

    private void ConfigureInputs()
    {
        ConfigurePort(_robotPort);
        ConfigurePort(_pcListenPort);
        ConfigureNumber(_maxBatch, 1, 12, 1);
        ConfigureDecimal(_testX, -9999, 9999, 2, 1);
        ConfigureDecimal(_testY, -9999, 9999, 2, 1);
        ConfigureDecimal(_testAngle, -360, 360, 2, 1);
    }

    private void LoadSettings()
    {
        _mode.SelectedIndex = _settings.IsServerMode ? 0 : 1;
        _robotIp.Text = _settings.RobotIP;
        _robotPort.Value = ClampDecimal(_settings.RobotPort, _robotPort.Minimum, _robotPort.Maximum);
        _pcListenIp.Text = _settings.PcListenIP;
        _pcListenPort.Value = ClampDecimal(_settings.PcListenPort, _pcListenPort.Minimum, _pcListenPort.Maximum);
        _protocol.SelectedIndex = _settings.UseBinaryBatchProtocol ? 0 : 1;
        _restrictIp.Checked = _settings.RestrictToRobotIP;
        _bigEndian.Checked = _settings.UseBigEndianRobotProtocol;
        _maxBatch.Value = ClampDecimal(_settings.MaxResultsPerRobotRequest, _maxBatch.Minimum, _maxBatch.Maximum);
        _testX.Value = ClampDecimal((decimal)_settings.TestCoordinateX, _testX.Minimum, _testX.Maximum);
        _testY.Value = ClampDecimal((decimal)_settings.TestCoordinateY, _testY.Minimum, _testY.Maximum);
        _testAngle.Value = ClampDecimal((decimal)_settings.TestCoordinateAngle, _testAngle.Minimum, _testAngle.Maximum);
        _serverTest.Checked = _settings.UseServerTestCoordinates;
        _enableFlask.Checked = _settings.EnableFlaskApi;
        _flaskUrl.Text = _settings.FlaskApiUrl;
        _enableDb.Checked = _settings.EnableLocalDatabase;
        _databasePath.Text = _settings.DatabasePath;
    }

    private void SaveSettings()
    {
        _settings.ConnectionMode = _mode.SelectedIndex == 1 ? RobotConnectionSettings.ModeClient : RobotConnectionSettings.ModeServer;
        _settings.RobotIP = _robotIp.Text.Trim();
        _settings.RobotPort = (int)_robotPort.Value;
        _settings.PcListenIP = _pcListenIp.Text.Trim();
        _settings.PcListenPort = (int)_pcListenPort.Value;
        _settings.CoordinateProtocol = _protocol.SelectedIndex == 0
            ? RobotConnectionSettings.ProtocolNachiBinaryBatch
            : RobotConnectionSettings.ProtocolAsciiLine;
        _settings.RestrictToRobotIP = _restrictIp.Checked;
        _settings.UseBigEndianRobotProtocol = _bigEndian.Checked;
        _settings.MaxResultsPerRobotRequest = (int)_maxBatch.Value;
        _settings.TestCoordinateX = (double)_testX.Value;
        _settings.TestCoordinateY = (double)_testY.Value;
        _settings.TestCoordinateAngle = (double)_testAngle.Value;
        _settings.UseServerTestCoordinates = _serverTest.Checked;
        _settings.EnableFlaskApi = _enableFlask.Checked;
        _settings.FlaskApiUrl = _flaskUrl.Text.Trim();
        _settings.EnableLocalDatabase = _enableDb.Checked;
        _settings.DatabasePath = _databasePath.Text.Trim();
        _settings.Save();
    }

    private void BrowseDatabasePath()
    {
        using var dialog = new SaveFileDialog
        {
            Title = "Select SQLite database",
            Filter = "SQLite database (*.db)|*.db|All files (*.*)|*.*",
            FileName = string.IsNullOrWhiteSpace(_databasePath.Text) ? "vision_results.db" : Path.GetFileName(_databasePath.Text)
        };

        string? currentDirectory = Path.GetDirectoryName(_databasePath.Text.Trim());
        if (!string.IsNullOrWhiteSpace(currentDirectory) && Directory.Exists(currentDirectory))
            dialog.InitialDirectory = currentDirectory;

        if (dialog.ShowDialog(this) == DialogResult.OK)
            _databasePath.Text = dialog.FileName;
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
            Padding = new Padding(8, 6, 8, 8)
        };
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, LabelColumnPercent));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100 - LabelColumnPercent));
        for (int i = 0; i < rowCount; i++)
            grid.RowStyles.Add(new RowStyle(SizeType.Absolute, RowHeight));
        return grid;
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

    private static void ConfigurePort(NumericUpDown input)
    {
        ConfigureNumber(input, 1, 65535, 1);
    }

    private static void ConfigureNumber(NumericUpDown input, decimal min, decimal max, decimal increment)
    {
        input.Minimum = min;
        input.Maximum = max;
        input.Increment = increment;
    }

    private static void ConfigureDecimal(NumericUpDown input, decimal min, decimal max, int decimals, decimal increment)
    {
        input.Minimum = min;
        input.Maximum = max;
        input.DecimalPlaces = decimals;
        input.Increment = increment;
    }

    private static decimal ClampDecimal(decimal value, decimal min, decimal max)
    {
        return Math.Min(max, Math.Max(min, value));
    }
}
