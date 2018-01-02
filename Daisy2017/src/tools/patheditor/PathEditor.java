package tools.patheditor;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.SwerveModifier;
import jaci.pathfinder.modifiers.TankModifier;
import java.awt.Color;
import java.awt.Component;
import java.awt.EventQueue;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.StringTokenizer;

import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JScrollPane;
import javax.swing.JSeparator;
import javax.swing.JTabbedPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.KeyStroke;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.table.AbstractTableModel;
import missdaisy.utilities.DaisyMath;
import org.eclipse.wb.swing.FocusTraversalOnArray;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartTheme;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.StandardChartTheme;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.Align;

public class PathEditor {

  // GUI Parameters
  private JFrame frmPathEditor;
  private JTable tableWaypoints;
  private JTextField textVelocity;
  private JTextField textAccel;
  private JTextField textJerk;

  JFreeChart chartPath;
  JFreeChart chartPath2;
  
  ChartPanel cpPath;
  ChartPanel cpMotion;
  ChartPanel cpHeading;

  JFileChooser fChooser;
  String filePath = "src/Config";

  final DecimalFormat fmt = new DecimalFormat("#0.000");

  // Algorithm Parameters
  XYSeriesCollection datasetPath;
  XYSeriesCollection datasetMotion;
  XYSeriesCollection datasetHeading;
  Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
      Trajectory.Config.SAMPLES_HIGH,
      0.005, 3.0, 6.0, 12.0);
  Trajectory trajectory;
  Trajectory[] driveTrajectories = new Trajectory[4];
  MyWaypointTable tableData;
  String RobotModifier = "Basic";
  String selectedLengthUnit = "in";
  String selectedSpeedUnit = "ft/s";
  String selectedAngleUnit = "deg";
  Boolean isRedSide = true;

  Waypoint[] waypoints = new Waypoint[]{new Waypoint(0, 0, 0), new Waypoint(32, -20, 0),
      new Waypoint(64, 0, 0)};
  private JTextField textWidth;
  private JTextField textLength;

  class MyWaypointTable extends AbstractTableModel {

    private static final long serialVersionUID = 3277070246373250747L;
    private String[] columnNames = {"Waypoint #", "X", "Y", "Heading"};
    private String[] units = {"none", "ft", "ft", "deg"};
    private ArrayList<ArrayList<Double>> data = new ArrayList<ArrayList<Double>>();

    public MyWaypointTable() {
      for (int i = 0; i < waypoints.length; i++) {
        ArrayList<Double> row = new ArrayList<Double>(
            Arrays.asList((double) i + 1, waypoints[i].x, waypoints[i].y, waypoints[i].angle));
        data.add(row);
      }
    }

    public ArrayList<ArrayList<Double>> getData() {
      return data;
    }

    public void Reset() {
      data = new ArrayList<ArrayList<Double>>();
      for (int i = 0; i < waypoints.length; i++) {
        ArrayList<Double> row = new ArrayList<Double>(
            Arrays.asList((double) i + 1, waypoints[i].x, waypoints[i].y, waypoints[i].angle));
        data.add(row);
      }
      fireTableDataChanged();
    }

    @Override
    public String getColumnName(int column) {
      return columnNames[column];
    }

    @Override
    public int getColumnCount() {
      return columnNames.length;
    }

    @Override
    public int getRowCount() {
      return data.size();
    }

    @Override
    public Double getValueAt(int row, int col) {
      return data.get(row).get(col);
    }

    @Override
    public boolean isCellEditable(int row, int col) {
      // Note that the data/cell address is constant,
      // no matter where the cell appears onscreen.
      if (col < 1) {
        return false;
      } else {
        return true;
      }
    }

    @Override
    public void setValueAt(Object value, int row, int col) {
      Double val = Double.parseDouble((String) value);
      // data[row][col] = val;
      data.get(row).set(col, val);
      fireTableCellUpdated(row, col);
    }

    public void addRow() {
      double numWaypoints = data.get(data.size() - 1).get(0);
      data.add(new ArrayList<Double>(Arrays.asList(numWaypoints + 1.0, 0.0, 0.0, 0.0)));
      fireTableDataChanged();
      tableWaypoints.getSelectionModel().setSelectionInterval(data.size() - 1, data.size() - 1);
    }

    public void insertRow(int row) {
      data.add(row, new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0, 0.0)));

      // Have to go through and re-label each waypoint #
      for (int i = 0; i < data.size(); i++) {
        data.get(i).set(0, (double) i + 1);
      }
      fireTableDataChanged();

      tableWaypoints.getSelectionModel().setSelectionInterval(Math.min(data.size() - 1, row),
          Math.min(data.size() - 1, row));
    }

    public void removeRow(int row) {
      if (data.size() == 1) {
        return;
      }
      data.remove(row);

      // Have to go through and re-label each waypoint #
      for (int i = 0; i < data.size(); i++) {
        data.get(i).set(0, (double) i + 1);
      }
      fireTableDataChanged();

      tableWaypoints.getSelectionModel().setSelectionInterval(Math.min(data.size() - 1, row),
          Math.min(data.size() - 1, row));

    }

    public Waypoint[] returnAsWaypoints() {
      Waypoint[] newWaypoints = new Waypoint[this.getRowCount()];

      for (int i = 0; i < this.getRowCount(); i++) {
        // newWaypoints[i] = new Waypoint(data[i][1], data[i][2],
        // Pathfinder.d2r(data[i][3]));
        newWaypoints[i] = new Waypoint(ConvertLength(data.get(i).get(1), selectedLengthUnit, "m"),
            ConvertLength(data.get(i).get(2), selectedLengthUnit, "m"),
            ConvertAngle(data.get(i).get(3), selectedAngleUnit, "rad"));
      }
      return newWaypoints;
    }

    public Waypoint[] returnRotatedWaypoints() {
      Waypoint[] newWaypoints = new Waypoint[this.getRowCount()];

      double angle = ConvertAngle(data.get(0).get(3), selectedAngleUnit, "rad");
      double cosAng = Math.cos(angle);
      double sinAng = Math.sin(angle);
      for (int i = 0; i < this.getRowCount(); i++) {
        double dx = data.get(i).get(1) - data.get(0).get(1);
        double dy = data.get(i).get(2) - data.get(0).get(2);
        double x = dx * cosAng + dy * sinAng;
        double y = -dx * sinAng + dy * cosAng;
        newWaypoints[i] = new Waypoint(ConvertLength(x, selectedLengthUnit, "m"),
            ConvertLength(y, selectedLengthUnit, "m"), DaisyMath.boundAngleNegPiToPiRadians(
            ConvertAngle(data.get(i).get(3) - data.get(0).get(3), selectedAngleUnit, "rad")));

      }
      return newWaypoints;
    }

    public Waypoint getFirstWaypoint() {
      Waypoint newWaypoints = new Waypoint(
          ConvertLength(data.get(0).get(1), selectedLengthUnit, "m"),
          ConvertLength(data.get(0).get(2), selectedLengthUnit, "m"),
          ConvertAngle(data.get(0).get(3), selectedAngleUnit, "rad"));
      return newWaypoints;
    }
  }
  
  /** 
   * GUI Handles
   */
  // Spline type
  JCheckBoxMenuItem chckbxmntmCubic;
  JCheckBoxMenuItem chckbxmntmNewQuintic;
  
  // Alliance Side
  JCheckBoxMenuItem chckbxmntmRedAlliance;
  JCheckBoxMenuItem chckbxmntmBlueAlliance;
  
  // Robot Type
  JCheckBoxMenuItem chckbxmntmBasic;
  JCheckBoxMenuItem chckbxmntmTank;
  JCheckBoxMenuItem chckbxmntmSwerve;
  
  // Length Units
  JCheckBoxMenuItem chckbxmntmInches;
  JCheckBoxMenuItem chckbxmntmFeet;
  JCheckBoxMenuItem chckbxmntmYards;
  JCheckBoxMenuItem chckbxmntmMeters;
  
  // Speed Units
  JCheckBoxMenuItem chckbxmntmFts;
  JCheckBoxMenuItem chckbxmntmMs;
  
  // Angle Units
  JCheckBoxMenuItem chckbxmntmRadians;
  JCheckBoxMenuItem chckbxmntmDegrees;

  /**
   * Launch the application.
   */
  public static void main(String[] args) {
    EventQueue.invokeLater(new Runnable() {
      @Override
      public void run() {
        try {
          PathEditor window = new PathEditor();
          window.frmPathEditor.setVisible(true);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    });
  }

  /**
   * Create the application.
   */
  public PathEditor() {
    initialize();
    GeneratePath();
  }

  /**
   * Initialize the contents of the frame.
   */
  private void initialize() {
    frmPathEditor = new JFrame();
    frmPathEditor.setResizable(false);
    frmPathEditor.setTitle("Path Editor");
    frmPathEditor.setBounds(100, 100, 650, 700);
    frmPathEditor.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frmPathEditor.getContentPane().setLayout(null);

    JButton btnGeneratePath = new JButton("Generate \r\nPath");
    btnGeneratePath.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        GeneratePath();
      }
    });
    btnGeneratePath.setBounds(357, 596, 277, 44);
    frmPathEditor.getContentPane().add(btnGeneratePath);

    JButton btnAddWaypoint = new JButton("Add");
    btnAddWaypoint.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        tableData.addRow();
      }
    });
    btnAddWaypoint.setBounds(357, 472, 85, 23);
    frmPathEditor.getContentPane().add(btnAddWaypoint);

    JButton btnRemoveWaypoint = new JButton("Remove");
    btnRemoveWaypoint.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        tableData.removeRow(tableWaypoints.getSelectedRow());
      }
    });
    btnRemoveWaypoint.setBounds(357, 524, 85, 23);
    frmPathEditor.getContentPane().add(btnRemoveWaypoint);

    JButton btnInsert = new JButton("Insert");
    btnInsert.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        tableData.insertRow(tableWaypoints.getSelectedRow());
      }
    });
    btnInsert.setBounds(357, 498, 85, 23);
    frmPathEditor.getContentPane().add(btnInsert);

    JLabel lblWaypoints = new JLabel("Waypoints");
    lblWaypoints.setFont(new Font("Tahoma", Font.BOLD, 11));
    lblWaypoints.setHorizontalAlignment(SwingConstants.CENTER);
    lblWaypoints.setBounds(10, 453, 337, 14);
    frmPathEditor.getContentPane().add(lblWaypoints);

    JTabbedPane tabbedPane = new JTabbedPane(SwingConstants.TOP);
    tabbedPane.setBounds(10, 11, 624, 431);
    frmPathEditor.getContentPane().add(tabbedPane);

    JMenuBar menuBar = new JMenuBar();
    frmPathEditor.setJMenuBar(menuBar);

    JMenu mfile = new JMenu("File");
    menuBar.add(mfile);

    JMenuItem mntmNew = new JMenuItem("New");
    mntmNew.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        ResetGUI();
      }
    });
    mntmNew.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_N, InputEvent.CTRL_MASK));
    mfile.add(mntmNew);

    JMenuItem mntmLoad = new JMenuItem("Load");
    mntmLoad.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        LoadTrajectory();
      }
    });
    mntmLoad.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, InputEvent.CTRL_MASK));
    mfile.add(mntmLoad);

    JSeparator separator = new JSeparator();
    mfile.add(separator);

    JMenuItem mntmSave = new JMenuItem("Save");
    mntmSave.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        SaveTrajectory();
      }
    });
    mntmSave.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, InputEvent.CTRL_MASK));
    mfile.add(mntmSave);

    JSeparator separator_1 = new JSeparator();
    mfile.add(separator_1);

    JMenuItem mntmExit = new JMenuItem("Exit");
    mntmExit.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_Q, InputEvent.CTRL_MASK));
    mntmExit.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        System.exit(JFrame.EXIT_ON_CLOSE);
      }
    });
    mfile.add(mntmExit);

    JMenu mnOptions = new JMenu("Options");
    menuBar.add(mnOptions);

    chckbxmntmCubic = new JCheckBoxMenuItem("Hermite Cubic");
    chckbxmntmCubic.setSelected(true);
    mnOptions.add(chckbxmntmCubic);

    chckbxmntmNewQuintic = new JCheckBoxMenuItem("Hermite Quintic");
    mnOptions.add(chckbxmntmNewQuintic);

    chckbxmntmCubic.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmCubic.setSelected(true);
        chckbxmntmNewQuintic.setSelected(false);
        config.fit = Trajectory.FitMethod.HERMITE_CUBIC;
        GeneratePath();
      }
    });

    chckbxmntmNewQuintic.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmCubic.setSelected(false);
        chckbxmntmNewQuintic.setSelected(true);
        config.fit = Trajectory.FitMethod.HERMITE_QUINTIC;
        GeneratePath();
      }
    });

    JSeparator separator_2 = new JSeparator();
    mnOptions.add(separator_2);

    chckbxmntmBasic = new JCheckBoxMenuItem("Basic");
    chckbxmntmBasic.setSelected(true);
    mnOptions.add(chckbxmntmBasic);

    chckbxmntmTank = new JCheckBoxMenuItem("Tank");
    mnOptions.add(chckbxmntmTank);

    chckbxmntmSwerve = new JCheckBoxMenuItem("Swerve");
    mnOptions.add(chckbxmntmSwerve);

    JSeparator separator_3 = new JSeparator();
    mnOptions.add(separator_3);

    chckbxmntmRedAlliance = new JCheckBoxMenuItem("Red Alliance");
    chckbxmntmRedAlliance.setSelected(true);
    mnOptions.add(chckbxmntmRedAlliance);

    chckbxmntmBlueAlliance = new JCheckBoxMenuItem("Blue Alliance");
    mnOptions.add(chckbxmntmBlueAlliance);

    chckbxmntmRedAlliance.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        chckbxmntmRedAlliance.setSelected(true);
        chckbxmntmBlueAlliance.setSelected(false);
        isRedSide = true;
        GeneratePath();
      }
    });

    chckbxmntmBlueAlliance.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        chckbxmntmRedAlliance.setSelected(false);
        chckbxmntmBlueAlliance.setSelected(true);
        isRedSide = false;
        GeneratePath();
      }
    });

    chckbxmntmBasic.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmBasic.setSelected(true);
        chckbxmntmTank.setSelected(false);
        chckbxmntmSwerve.setSelected(false);
        RobotModifier = "Basic";
        GeneratePath();
      }
    });

    chckbxmntmTank.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmBasic.setSelected(false);
        chckbxmntmTank.setSelected(true);
        chckbxmntmSwerve.setSelected(false);
        RobotModifier = "Tank";
        GeneratePath();
      }
    });

    chckbxmntmSwerve.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmBasic.setSelected(false);
        chckbxmntmTank.setSelected(false);
        chckbxmntmSwerve.setSelected(true);
        RobotModifier = "Swerve";
        GeneratePath();
      }
    });

    JMenu mnUnits = new JMenu("Units");
    menuBar.add(mnUnits);

    JMenu mnLength = new JMenu("Length");
    mnUnits.add(mnLength);

    chckbxmntmInches = new JCheckBoxMenuItem("Inches");
    chckbxmntmInches.setSelected(true);
    mnLength.add(chckbxmntmInches);

    chckbxmntmFeet = new JCheckBoxMenuItem("Feet");
    mnLength.add(chckbxmntmFeet);

    chckbxmntmYards = new JCheckBoxMenuItem("Yards");
    mnLength.add(chckbxmntmYards);

    chckbxmntmMeters = new JCheckBoxMenuItem("Meters");
    mnLength.add(chckbxmntmMeters);

    chckbxmntmInches.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmInches.setSelected(true);
        chckbxmntmFeet.setSelected(false);
        chckbxmntmYards.setSelected(false);
        chckbxmntmMeters.setSelected(false);
        String oldUnits = selectedLengthUnit;
        selectedLengthUnit = "in";
        UpdateRobotSize(oldUnits, selectedLengthUnit);
        UpdateUnits();
      }
    });

    chckbxmntmFeet.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmInches.setSelected(false);
        chckbxmntmFeet.setSelected(true);
        chckbxmntmYards.setSelected(false);
        chckbxmntmMeters.setSelected(false);
        String oldUnits = selectedLengthUnit;
        selectedLengthUnit = "ft";
        UpdateRobotSize(oldUnits, selectedLengthUnit);
        UpdateUnits();
      }
    });

    chckbxmntmYards.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmInches.setSelected(false);
        chckbxmntmFeet.setSelected(false);
        chckbxmntmYards.setSelected(true);
        chckbxmntmMeters.setSelected(false);
        String oldUnits = selectedLengthUnit;
        selectedLengthUnit = "yd";
        UpdateRobotSize(oldUnits, selectedLengthUnit);
        UpdateUnits();
      }
    });

    chckbxmntmMeters.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmInches.setSelected(false);
        chckbxmntmFeet.setSelected(false);
        chckbxmntmYards.setSelected(false);
        chckbxmntmMeters.setSelected(true);
        String oldUnits = selectedLengthUnit;
        selectedLengthUnit = "m";
        UpdateRobotSize(oldUnits, selectedLengthUnit);
        UpdateUnits();
      }
    });

    JMenu mnSpeed = new JMenu("Speed");
    mnUnits.add(mnSpeed);

    chckbxmntmFts = new JCheckBoxMenuItem("ft/s");
    chckbxmntmFts.setSelected(true);
    mnSpeed.add(chckbxmntmFts);

    chckbxmntmMs = new JCheckBoxMenuItem("m/s");
    mnSpeed.add(chckbxmntmMs);

    chckbxmntmFts.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmFts.setSelected(true);
        chckbxmntmMs.setSelected(false);
        String oldUnit = selectedSpeedUnit;
        selectedSpeedUnit = "ft/s";
        UpdateRobotSpeeds(oldUnit, selectedSpeedUnit);
        UpdateUnits();
      }
    });

    chckbxmntmMs.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmFts.setSelected(false);
        chckbxmntmMs.setSelected(true);
        String oldUnit = selectedSpeedUnit;
        selectedSpeedUnit = "m/s";
        UpdateRobotSpeeds(oldUnit, selectedSpeedUnit);
        UpdateUnits();
      }
    });

    JMenu mnAngle = new JMenu("Angle");
    mnUnits.add(mnAngle);

    chckbxmntmRadians = new JCheckBoxMenuItem("Radians");
    mnAngle.add(chckbxmntmRadians);

    chckbxmntmDegrees = new JCheckBoxMenuItem("Degrees");
    chckbxmntmDegrees.setSelected(true);
    mnAngle.add(chckbxmntmDegrees);

    chckbxmntmRadians.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmRadians.setSelected(true);
        chckbxmntmDegrees.setSelected(false);
        selectedAngleUnit = "rad";
        UpdateUnits();
      }
    });

    chckbxmntmDegrees.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        chckbxmntmRadians.setSelected(false);
        chckbxmntmDegrees.setSelected(true);
        selectedAngleUnit = "deg";
        UpdateUnits();
      }
    });
    
    JMenu mnDisplay = new JMenu("Display");
    menuBar.add(mnDisplay);
    
    JMenuItem mnResetDisplay = new JMenuItem("Reset");
    mnResetDisplay.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent arg0) {
        SetAxisLimits2017();
      }
    });
    mnResetDisplay.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_R, InputEvent.CTRL_MASK));
    mnDisplay.add(mnResetDisplay);
    

    JLabel lblMaxVel = new JLabel("Max Vel:");
    lblMaxVel.setHorizontalAlignment(SwingConstants.RIGHT);
    lblMaxVel.setBounds(448, 466, 70, 14);
    frmPathEditor.getContentPane().add(lblMaxVel);

    JLabel lblMaxAccel = new JLabel("Max Accel:");
    lblMaxAccel.setHorizontalAlignment(SwingConstants.RIGHT);
    lblMaxAccel.setBounds(448, 491, 70, 14);
    frmPathEditor.getContentPane().add(lblMaxAccel);

    JLabel lblMaxJerk = new JLabel("Max Jerk:");
    lblMaxJerk.setHorizontalAlignment(SwingConstants.RIGHT);
    lblMaxJerk.setBounds(448, 516, 70, 14);
    frmPathEditor.getContentPane().add(lblMaxJerk);

    textVelocity = new JTextField();
    textVelocity.setHorizontalAlignment(SwingConstants.RIGHT);
    textVelocity.setText("10.0");
    textVelocity.setBounds(522, 466, 112, 20);
    frmPathEditor.getContentPane().add(textVelocity);
    textVelocity.setColumns(10);

    textAccel = new JTextField();
    textAccel.setHorizontalAlignment(SwingConstants.RIGHT);
    textAccel.setText("20.0");
    textAccel.setBounds(522, 491, 112, 20);
    frmPathEditor.getContentPane().add(textAccel);
    textAccel.setColumns(10);

    textJerk = new JTextField();
    textJerk.setHorizontalAlignment(SwingConstants.RIGHT);
    textJerk.setText("40.0");
    textJerk.setBounds(522, 517, 112, 20);
    frmPathEditor.getContentPane().add(textJerk);
    textJerk.setColumns(10);

    JLabel lblRobotParameters = new JLabel("Robot Parameters");
    lblRobotParameters.setFont(new Font("Tahoma", Font.BOLD, 11));
    lblRobotParameters.setHorizontalAlignment(SwingConstants.CENTER);
    lblRobotParameters.setBounds(452, 450, 182, 14);
    frmPathEditor.getContentPane().add(lblRobotParameters);

    JLabel lblRobotWidth = new JLabel("Robot Width:");
    lblRobotWidth.setHorizontalAlignment(SwingConstants.RIGHT);
    lblRobotWidth.setBounds(432, 541, 85, 14);
    frmPathEditor.getContentPane().add(lblRobotWidth);

    textWidth = new JTextField();
    textWidth.setHorizontalAlignment(SwingConstants.RIGHT);
    textWidth.setText("31");
    textWidth.setBounds(522, 541, 112, 20);
    frmPathEditor.getContentPane().add(textWidth);
    textWidth.setColumns(10);

    textLength = new JTextField();
    textLength.setText("23.5");
    textLength.setHorizontalAlignment(SwingConstants.RIGHT);
    textLength.setBounds(522, 565, 112, 20);
    frmPathEditor.getContentPane().add(textLength);
    textLength.setColumns(10);

    JLabel lblRobotLength = new JLabel("Robot Length:");
    lblRobotLength.setHorizontalAlignment(SwingConstants.RIGHT);
    lblRobotLength.setBounds(432, 566, 86, 14);
    frmPathEditor.getContentPane().add(lblRobotLength);

    tableData = new MyWaypointTable();
    JScrollPane scrollPane = new JScrollPane();
    scrollPane.setBounds(10, 472, 337, 168);
    frmPathEditor.getContentPane().add(scrollPane);
    tableWaypoints = new JTable(tableData);
    scrollPane.setViewportView(tableWaypoints);
    tableWaypoints.setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
    tableWaypoints.setForeground(Color.BLACK);

    tableWaypoints.getSelectionModel().setSelectionInterval(0, 0);

    // Initialize the datasets
    datasetPath = new XYSeriesCollection();
    datasetMotion = new XYSeriesCollection();
    datasetHeading = new XYSeriesCollection();

    // Create the graphs
    /*
    chartPath = ChartFactory.createXYLineChart("Generated Path", "Down Field (in)",
        "Driver Station (in)", datasetPath);
    chartPath.getXYPlot().getRangeAxis().setInverted(true);
    */
    
    NumberAxis xAxis = new NumberAxis("Down Field (in)");
    xAxis.setAutoRangeIncludesZero(false);
    
    NumberAxis yAxis = new NumberAxis("Driver Station (in)");
    yAxis.setInverted(true);
    
    XYItemRenderer renderer = new XYLineAndShapeRenderer(true, false);
    
    XYPlot plot = new XYPlotWithZoomableBackgroundImage(datasetPath, xAxis, yAxis, renderer, true);
    plot.setOrientation(PlotOrientation.VERTICAL);
    
    chartPath = new JFreeChart("Generated Path", JFreeChart.DEFAULT_TITLE_FONT, plot, false);
    ChartTheme currentTheme = new StandardChartTheme("JFree");
    currentTheme.apply(chartPath);
    
    JFreeChart chartMotion = ChartFactory.createXYLineChart("Motion Profile", "Time (s)",
        "Speed (m/s),  Accel (m/s^2)", datasetMotion);
    JFreeChart chartHeading = ChartFactory.createXYLineChart("Heading", "Time (s)", "Heading (deg)",
        datasetHeading);
    cpMotion = new ChartPanel(chartMotion);

    // Stick the graphs to a chart panel
    cpPath = new ChartPanel(chartPath);

    // Add the chart panels to the tab panels
    tabbedPane.add(cpPath, "Trajectory");

    frmPathEditor.setFocusTraversalPolicy(new FocusTraversalOnArray(new Component[] {
        btnGeneratePath, frmPathEditor.getContentPane(), tabbedPane, lblWaypoints, tableWaypoints,
        btnAddWaypoint, cpPath, btnRemoveWaypoint, cpMotion, menuBar, mfile, mntmNew, mntmLoad,
        separator, mntmSave, separator_1, mntmExit, mnOptions, chckbxmntmCubic,
        chckbxmntmNewQuintic, separator_2, chckbxmntmBasic, chckbxmntmTank, chckbxmntmSwerve}));
            
            
    cpHeading = new ChartPanel(chartHeading);
    tabbedPane.add(cpHeading, "Heading");
    tabbedPane.add(cpMotion, "Motion Profile");

  }

  public void UpdateUnits() {
    cpPath.getChart().getXYPlot().getDomainAxis()
        .setLabel("Down Field (" + selectedLengthUnit + ")");
    cpPath.getChart().getXYPlot().getRangeAxis()
        .setLabel("Driver Station (" + selectedLengthUnit + ")");

    cpHeading.getChart().getXYPlot().getRangeAxis().setLabel("Heading (" + selectedAngleUnit + ")");

    cpMotion.getChart().getXYPlot().getRangeAxis()
        .setLabel("Speed (" + selectedSpeedUnit + "),  Accel (" + selectedSpeedUnit + "^2)");

    GeneratePath();
  }

  public void UpdateRobotSpeeds(String from, String to) {
    Double val = Double.parseDouble(textVelocity.getText());
    textVelocity.setText(fmt.format(ConvertSpeed(val, from, to)));

    val = Double.parseDouble(textAccel.getText());
    textAccel.setText(fmt.format(ConvertSpeed(val, from, to)));

    val = Double.parseDouble(textJerk.getText());
    textJerk.setText(fmt.format(ConvertSpeed(val, from, to)));
  }

  public void UpdateRobotSize(String from, String to) {
    Double val = Double.parseDouble(textWidth.getText());
    textWidth.setText(fmt.format(ConvertLength(val, from, to)));

    val = Double.parseDouble(textLength.getText());
    textLength.setText(fmt.format(ConvertLength(val, from, to)));
  }

  public void GeneratePath() {
    // Configure the path generator parameters
    config.max_velocity = ConvertSpeed(Double.parseDouble(textVelocity.getText()),
        selectedSpeedUnit, "m/s");
    config.max_acceleration = ConvertSpeed(Double.parseDouble(textAccel.getText()),
        selectedSpeedUnit, "m/s");
    config.max_jerk = ConvertSpeed(Double.parseDouble(textJerk.getText()), selectedSpeedUnit,
        "m/s");

    // Genereate a trajectory using splines
    waypoints = tableData.returnRotatedWaypoints();
    trajectory = Pathfinder.generate(waypoints, config);

    // Clear graphs of previous plot lines
    datasetPath.removeAllSeries();
    datasetMotion.removeAllSeries();
    datasetHeading.removeAllSeries();

    // Draw the field
    Draw2017Field();

    // Plot the waypoints on the Trajectory plot
    XYSeries waypointXY = new XYSeries("Waypoints", false, true);
    Waypoint firstWP = tableData.getFirstWaypoint();
    double angle = firstWP.angle;
    double cosAng = Math.cos(angle);
    double sinAng = Math.sin(angle);
    for (int j = 0; j < waypoints.length; j++) {
      double x = waypoints[j].x * cosAng - waypoints[j].y * sinAng + firstWP.x;
      double y = waypoints[j].x * sinAng + waypoints[j].y * cosAng + firstWP.y;
      waypointXY.add(ConvertLength(x, "m", selectedLengthUnit),
          ConvertLength(y, "m", selectedLengthUnit));
    }
    datasetPath.addSeries(waypointXY);

    // Generate the trajectories based on the robot modifier
    double rWidth = ConvertLength(Double.parseDouble(textWidth.getText()), selectedLengthUnit, "m");
    double rLength = ConvertLength(Double.parseDouble(textLength.getText()), selectedLengthUnit,
        "m");
    switch (RobotModifier) {
      case "Basic":
        driveTrajectories[0] = trajectory;

        GenerateDataSeries("", driveTrajectories[0]);
        break;
      case "Tank":
        TankModifier tankModifier = new TankModifier(trajectory).modify(rWidth);

        driveTrajectories[0] = tankModifier.getLeftTrajectory();
        driveTrajectories[1] = tankModifier.getRightTrajectory();

        GenerateDataSeries("Path", trajectory);
        GenerateDataSeries("Left", driveTrajectories[0]);
        GenerateDataSeries("Right", driveTrajectories[1]);
        break;
      case "Swerve":
        SwerveModifier swerveModifier = new SwerveModifier(trajectory).modify(rWidth, rLength,
            SwerveModifier.Mode.SWERVE_DEFAULT);

        driveTrajectories[0] = swerveModifier.getFrontLeftTrajectory();
        driveTrajectories[1] = swerveModifier.getFrontRightTrajectory();
        driveTrajectories[2] = swerveModifier.getBackLeftTrajectory();
        driveTrajectories[3] = swerveModifier.getBackRightTrajectory();

        GenerateDataSeries("Front Left", driveTrajectories[0]);
        GenerateDataSeries("Front Right", driveTrajectories[1]);
        GenerateDataSeries("Back Left", driveTrajectories[2]);
        GenerateDataSeries("Back Right", driveTrajectories[3]);
        break;
    }

    // Refresh the chart panels
    cpPath.repaint();
    cpMotion.repaint();
    cpHeading.repaint();

    // JOptionPane.showMessageDialog(null, "Path Generated!");
  }

  public void Draw2017Field() {
    // Reference used to figure out how to have a zoomable background image
    //http://www.jfree.org/phpBB2/viewtopic.php?f=3&t=17868&sid=936992de571bfe625c9dfa12d1476bcc
    
    BufferedImage image = null;
    File url = new File("src/tools/patheditor/2017Red.png");
    if (!isRedSide){
      url = new File("src/tools/patheditor/2017Blue.png");
    }
    try {
      image = ImageIO.read(url);
      Graphics2D g = image.createGraphics();
      
      chartPath.getPlot().setBackgroundImage(image);
      //chartPath.getPlot().setBackgroundAlpha((float) 0.0);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    SetAxisLimits2017();
    
    // Draw the field boarder
    XYSeries arenaXY = new XYSeries("Arena Border", false, true);
    double ax = ConvertLength(0.0, "in", selectedLengthUnit);
    double ay = ConvertLength(0.0, "in", selectedLengthUnit);
    double fx = ConvertLength(54.33333, "ft", selectedLengthUnit);
    double fy = ConvertLength(13.5, "ft", selectedLengthUnit);
    arenaXY.add(-ax, fy+ay);
    arenaXY.add(fx+ax, fy+ay);
    arenaXY.add(fx+ax, -fy-ay);
    arenaXY.add(-ax, -fy-ay);
    arenaXY.add(-ax, fy+ay);
    datasetPath.addSeries(arenaXY);
    
    
    // Draw the field boarder
    XYSeries borderXY = new XYSeries("Field Border", false, true);
    double x = ConvertLength(54.33333, "ft", selectedLengthUnit);
    double y = ConvertLength(13.5, "ft", selectedLengthUnit);
    borderXY.add(0, y);
    borderXY.add(x, y);
    borderXY.add(x, -y);
    borderXY.add(0, -y);
    borderXY.add(0, y);
    datasetPath.addSeries(borderXY);

    // Draw the airships
    double[] baseX = {0.0, 35.295, 70.59, 70.59, 35.295, 0.0, 0.0};
    double[] baseY = {20, 40, 20, -20, -40, -20, 20};

    XYSeries redAirshapeBaseXY = new XYSeries("Red Airship Base", false, true);
    x = ConvertLength(114.3, "in", selectedLengthUnit);
    y = ConvertLength(0.0, "in", selectedLengthUnit);
    for (int i = 0; i < baseX.length; i++) {
      redAirshapeBaseXY.add(ConvertLength(baseX[i], "in", selectedLengthUnit) + x,
          ConvertLength(baseY[i], "in", selectedLengthUnit) + y);
    }
    datasetPath.addSeries(redAirshapeBaseXY);

    XYSeries blueAirshapeBaseXY = new XYSeries("Blue Airship Base", false, true);
    x = ConvertLength(54.33333, "ft", selectedLengthUnit) - ConvertLength(114.3 + 70.59, "in",
        selectedLengthUnit);
    y = ConvertLength(0.0, "in", selectedLengthUnit);
    for (int i = 0; i < baseX.length; i++) {
      blueAirshapeBaseXY.add(ConvertLength(baseX[i], "in", selectedLengthUnit) + x,
          ConvertLength(baseY[i], "in", selectedLengthUnit) + y);
    }
    datasetPath.addSeries(blueAirshapeBaseXY);

    // Draw the field lines
    XYSeries redKeyXY = new XYSeries("Red Key", false, true);
    XYSeries redZoneXY = new XYSeries("Red Retrieval Zone", false, true);
    XYSeries blueKeyXY = new XYSeries("Blue Key", false, true);
    XYSeries blueZoneXY = new XYSeries("Blue Retrieval Zone", false, true);
    if (isRedSide) {
      redKeyXY.add(0.0, ConvertLength(3.5, "ft", selectedLengthUnit));
      redKeyXY.add(ConvertLength(10.0, "ft", selectedLengthUnit),
          ConvertLength(13.5, "ft", selectedLengthUnit));

      redZoneXY.add(ConvertLength(54.33333, "ft", selectedLengthUnit),
          ConvertLength(-13.5 / 2 - 0.833333, "ft", selectedLengthUnit));
      redZoneXY.add(ConvertLength(54.33333 - 13.791666, "ft", selectedLengthUnit),
          ConvertLength(-13.5, "ft", selectedLengthUnit));

      blueKeyXY.add(ConvertLength(54.33333, "ft", selectedLengthUnit),
          ConvertLength(3.5, "ft", selectedLengthUnit));
      blueKeyXY.add(ConvertLength(54.33333 - 10.0, "ft", selectedLengthUnit),
          ConvertLength(13.5, "ft", selectedLengthUnit));

      blueZoneXY.add(ConvertLength(0.0, "ft", selectedLengthUnit),
          ConvertLength(-13.5 / 2 - 0.833333, "ft", selectedLengthUnit));
      blueZoneXY.add(ConvertLength(13.791666, "ft", selectedLengthUnit),
          ConvertLength(-13.5, "ft", selectedLengthUnit));

    } else {
      redKeyXY.add(ConvertLength(54.33333, "ft", selectedLengthUnit),
          ConvertLength(-3.5, "ft", selectedLengthUnit));
      redKeyXY.add(ConvertLength(54.33333 - 10.0, "ft", selectedLengthUnit),
          ConvertLength(-13.5, "ft", selectedLengthUnit));

      redZoneXY.add(ConvertLength(0.0, "ft", selectedLengthUnit),
          ConvertLength(13.5 / 2 + 0.833333, "ft", selectedLengthUnit));
      redZoneXY.add(ConvertLength(13.791666, "ft", selectedLengthUnit),
          ConvertLength(13.5, "ft", selectedLengthUnit));

      blueKeyXY.add(ConvertLength(0.0, "ft", selectedLengthUnit),
          ConvertLength(-3.5, "ft", selectedLengthUnit));
      blueKeyXY.add(ConvertLength(10.0, "ft", selectedLengthUnit),
          ConvertLength(-13.5, "ft", selectedLengthUnit));

      blueZoneXY.add(ConvertLength(54.33333, "ft", selectedLengthUnit),
          ConvertLength(13.5 / 2 + 0.833333, "ft", selectedLengthUnit));
      blueZoneXY.add(ConvertLength(54.33333 - 13.791666, "ft", selectedLengthUnit),
          ConvertLength(13.5, "ft", selectedLengthUnit));
    }

    datasetPath.addSeries(redKeyXY);
    datasetPath.addSeries(redZoneXY);
    datasetPath.addSeries(blueKeyXY);
    datasetPath.addSeries(blueZoneXY);

    // Set the line colors
    UpdateColors2017();
  }

  public void SetAxisLimits2017(){
    chartPath.getXYPlot().getRangeAxis().setInverted(true);
    if (isRedSide){
      //chartPath.getXYPlot().getDomainAxis().setRange(-35.0, 687);
      //chartPath.getXYPlot().getRangeAxis().setRange(-175.0, 175.0);
      chartPath.getXYPlot().getDomainAxis().setRange(0.0, 652);
      chartPath.getXYPlot().getRangeAxis().setRange(-162.0, 162.0);
    } else {
      chartPath.getXYPlot().getDomainAxis().setRange(0.0, 652);
      chartPath.getXYPlot().getRangeAxis().setRange(-162.0, 162.0);
      //chartPath.getXYPlot().getDomainAxis().setRange(-33.0, 670);
      //chartPath.getXYPlot().getRangeAxis().setRange(-172.0, 174.0);
    }
  }
  
  public void UpdateColors2017() {
    XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();

    int index = 0;
    // "0" is the arena border
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesPaint(index, Color.BLACK);
    renderer.setSeriesVisibleInLegend(index++, false);
    
    // "0" is the field border
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesPaint(index, Color.BLACK);
    renderer.setSeriesVisibleInLegend(index++, false);

    // "1" is the outline of the red airship base
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesVisibleInLegend(index++, false);

    // "2" is the outline of the blue airship base
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesPaint(index, Color.BLUE);
    renderer.setSeriesFillPaint(index, Color.BLUE);
    renderer.setSeriesVisibleInLegend(index++, false);

    // "3" is the outline of the red key
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesVisibleInLegend(index++, false);

    // "4" is the outline of the red retrieval zone
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesVisibleInLegend(index++, false);

    // "5" is the outline of the red key
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesVisibleInLegend(index++, false);

    // "6" is the outline of the red retrieval zone
    renderer.setSeriesLinesVisible(index, true);
    renderer.setSeriesShapesVisible(index, false);
    renderer.setSeriesVisibleInLegend(index++, false);

    if (isRedSide) {
      renderer.setSeriesPaint(2, Color.RED);
      renderer.setSeriesFillPaint(2, Color.RED);
      renderer.setSeriesPaint(3, Color.BLUE);
      renderer.setSeriesFillPaint(3, Color.BLUE);
      /*
			 * renderer.setSeriesPaint(5, Color.RED);
			 * renderer.setSeriesFillPaint(5, Color.RED);
			 * renderer.setSeriesPaint(6, Color.RED);
			 * renderer.setSeriesFillPaint(6, Color.RED);
			 * renderer.setSeriesPaint(7, Color.BLUE);
			 * renderer.setSeriesFillPaint(7, Color.BLUE);
			 * renderer.setSeriesPaint(8, Color.BLUE);
			 * renderer.setSeriesFillPaint(8, Color.BLUE);
			 */
    } else {
      renderer.setSeriesPaint(2, Color.BLUE);
      renderer.setSeriesFillPaint(2, Color.BLUE);
      renderer.setSeriesPaint(3, Color.RED);
      renderer.setSeriesFillPaint(3, Color.RED);
			/*
			 * renderer.setSeriesPaint(5, Color.BLUE);
			 * renderer.setSeriesFillPaint(5, Color.BLUE);
			 * renderer.setSeriesPaint(6, Color.BLUE);
			 * renderer.setSeriesFillPaint(6, Color.BLUE);
			 * renderer.setSeriesPaint(7, Color.RED);
			 * renderer.setSeriesFillPaint(7, Color.RED);
			 * renderer.setSeriesPaint(8, Color.RED);
			 * renderer.setSeriesFillPaint(8, Color.RED);
			 */
    }
    renderer.setSeriesPaint(4, Color.RED);
    renderer.setSeriesFillPaint(4, Color.RED);
    renderer.setSeriesPaint(5, Color.RED);
    renderer.setSeriesFillPaint(5, Color.RED);
    renderer.setSeriesPaint(6, Color.BLUE);
    renderer.setSeriesFillPaint(6, Color.BLUE);
    renderer.setSeriesPaint(7, Color.BLUE);
    renderer.setSeriesFillPaint(7, Color.BLUE);

    // "7" is the scatter plot of the waypoints
    renderer.setSeriesLinesVisible(8, true);
    renderer.setSeriesShapesVisible(8, true);
    renderer.setSeriesPaint(8, Color.GREEN);

    // "8" is the line plot of the trajectory
    renderer.setSeriesLinesVisible(9, true);
    renderer.setSeriesShapesVisible(9, false);
    renderer.setSeriesPaint(9, Color.CYAN);

    cpPath.getChart().getXYPlot().setRenderer(renderer);
  }

  public void GenerateDataSeries(String name, Trajectory trajectory) {

    // Populate the lines for each data type
    XYSeries posXY = new XYSeries(name + " Position", false, true);
    XYSeries vel = new XYSeries(name + " Velocity");
    XYSeries acc = new XYSeries(name + " Acceleration");
    XYSeries heading = new XYSeries(name + " Heading");

    Waypoint firstWP = tableData.getFirstWaypoint();
    double angle = firstWP.angle;
    double cosAng = Math.cos(angle);
    double sinAng = Math.sin(angle);
    for (int j = 0; j < trajectory.length(); j++) {
      double x = trajectory.get(j).x * cosAng - trajectory.get(j).y * sinAng + firstWP.x;
      double y = trajectory.get(j).x * sinAng + trajectory.get(j).y * cosAng + firstWP.y;
      posXY.add(ConvertLength(x, "m", selectedLengthUnit),
          ConvertLength(y, "m", selectedLengthUnit));

      vel.add(j * trajectory.segments[j].dt,
          ConvertSpeed(trajectory.segments[j].velocity, "m/s", selectedSpeedUnit));
      acc.add(j * trajectory.segments[j].dt,
          ConvertSpeed(trajectory.segments[j].acceleration, "m/s", selectedSpeedUnit));

      if (selectedAngleUnit == "rad") {
        heading.add(j * trajectory.segments[j].dt,
            DaisyMath.boundAngleNegPiToPiRadians(trajectory.segments[j].heading));
      } else {
        heading.add(j * trajectory.segments[j].dt,
            DaisyMath.boundAngleNeg180to180Degrees(
                ConvertAngle(trajectory.segments[j].heading, "rad", "deg")));
      }
    }

    // Add the data series to the datasets
    datasetPath.addSeries(posXY);

    datasetMotion.addSeries(vel);
    datasetMotion.addSeries(acc);

    datasetHeading.addSeries(heading);
  }

  public void ResetGUI() {
    waypoints = new Waypoint[]{new Waypoint(0, 0, 0), new Waypoint(1, 0, 0)};
    tableData.Reset();
    isRedSide = true;
    GeneratePath();
  }

  public void LoadTrajectory() {
    fChooser = new JFileChooser();
    fChooser.setCurrentDirectory(new java.io.File(filePath));
    fChooser.setDialogTitle("Load Path Segment");
    fChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
    fChooser.setFileFilter(new FileNameExtensionFilter("Text File", "txt"));
    fChooser.setAcceptAllFileFilterUsed(false);

    if (fChooser.showOpenDialog(frmPathEditor) == JFileChooser.APPROVE_OPTION) {

      System.out.println("Selected File: " + fChooser.getSelectedFile().toString());
      frmPathEditor.setTitle("Path Editor - " + fChooser.getSelectedFile().toString());
      BufferedReader mReader = null;
      try {

        // Open the new file
        File mFile = new File(fChooser.getSelectedFile().toString());
        if (!mFile.exists()) {
          // fileConnection.create();
          System.err.println("Could not find specified file!");
          return;
        }

        // Make an I/O adapter sandwich to actually get some text out
        mReader = new BufferedReader(new FileReader(mFile));

        // Now parse the thing
        String lLine;

        // Pathfinder Properties
        lLine = mReader.readLine();
        chckbxmntmCubic.setSelected(false);
        chckbxmntmNewQuintic.setSelected(false);
        switch (lLine) {
          case "HERMITE_CUBIC":
            config.fit = Trajectory.FitMethod.HERMITE_CUBIC;
            chckbxmntmCubic.setSelected(true);
            break;
          case "HERMITE_QUINTIC":
            config.fit = Trajectory.FitMethod.HERMITE_QUINTIC;
            chckbxmntmNewQuintic.setSelected(true);
            break;
          default:
            config.fit = Trajectory.FitMethod.HERMITE_CUBIC;
            chckbxmntmCubic.setSelected(true);
        }
        
        RobotModifier = mReader.readLine();
        chckbxmntmBasic.setSelected(false);
        chckbxmntmTank.setSelected(false);
        chckbxmntmSwerve.setSelected(false);
        switch (RobotModifier) {
          case "Tank":
            chckbxmntmTank.setSelected(true);
            break;
          case "Swerve":
            chckbxmntmSwerve.setSelected(true);
            break;
          default:
            // Basic
            chckbxmntmBasic.setSelected(true);
        }

        lLine = mReader.readLine();
        chckbxmntmRedAlliance.setSelected(false);
        chckbxmntmBlueAlliance.setSelected(false);
        switch (lLine) {
          case "true":
            isRedSide = true;
            chckbxmntmRedAlliance.setSelected(true);
            break;
          case "false":
            isRedSide = false;
            chckbxmntmBlueAlliance.setSelected(true);
            break;
          default:
            isRedSide = true;
            chckbxmntmRedAlliance.setSelected(true);
        }

        selectedLengthUnit = mReader.readLine();
        chckbxmntmInches.setSelected(false);
        chckbxmntmFeet.setSelected(false);
        chckbxmntmYards.setSelected(false);
        chckbxmntmMeters.setSelected(false);
        switch (selectedLengthUnit) {
          case "Feet":
            chckbxmntmFeet.setSelected(true);
            break;
          case "Yards":
            chckbxmntmYards.setSelected(true);
            break;
          case "Meters":
            chckbxmntmMeters.setSelected(true);
            break;
          default:
            // Inches
            chckbxmntmInches.setSelected(true);
        }
        
        selectedSpeedUnit = mReader.readLine();
        chckbxmntmFts.setSelected(false);
        chckbxmntmMs.setSelected(false);
        switch (selectedLengthUnit) {
          case "m/s":
            chckbxmntmMs.setSelected(true);
            break;
          default:
            // ft/s
            chckbxmntmFts.setSelected(true);
        }
        
        selectedAngleUnit = mReader.readLine();
        chckbxmntmRadians.setSelected(false);
        chckbxmntmDegrees.setSelected(false);
        switch (selectedAngleUnit) {
          case "deg":
            chckbxmntmDegrees.setSelected(true);
            break;
          default:
            // rad
            chckbxmntmRadians.setSelected(true);
        }

        lLine = mReader.readLine();
        StringTokenizer lStringTok = new StringTokenizer(lLine);
        int lNumTokens = lStringTok.countTokens();
        // We best have at least two tokens ...
        if (lNumTokens < 3) {
          // We best have at least two tokens ...
          throw new IOException("Malformed file");
        }

        for (int j = 0; j < 3; j++) {
          String xPosStr = lStringTok.nextToken();
          if (xPosStr.lastIndexOf(",") > 0) {
            xPosStr = xPosStr.substring(0, xPosStr.lastIndexOf(","));
          }

          switch (j) {
            case 0:
              textVelocity.setText(xPosStr);
              break;
            case 1:
              textAccel.setText(xPosStr);
              break;
            case 2:
              textJerk.setText(xPosStr);
              break;
          }
        }

        lLine = mReader.readLine();
        lStringTok = new StringTokenizer(lLine);
        lNumTokens = lStringTok.countTokens();
        // We best have at least two tokens ...
        if (lNumTokens < 2) {
          // We best have at least two tokens ...
          throw new IOException("Malformed file");
        }

        for (int j = 0; j < 2; j++) {
          String xPosStr = lStringTok.nextToken();
          if (xPosStr.lastIndexOf(",") > 0) {
            xPosStr = xPosStr.substring(0, xPosStr.lastIndexOf(","));
          }

          switch (j) {
            case 0:
              textWidth.setText(xPosStr);
              break;
            case 1:
              textLength.setText(xPosStr);
              break;
          }
        }

        String numWaypoints = mReader.readLine();
        waypoints = new Waypoint[Integer.parseInt(numWaypoints)];
        for (int i = 0; i < Double.parseDouble(numWaypoints); i++) {
          lLine = mReader.readLine();
          lStringTok = new StringTokenizer(lLine);
          lNumTokens = lStringTok.countTokens();
          // We best have at least two tokens ...
          if (lNumTokens < 3) {
            throw new IOException("Malformed file");
          }

          Double[] val = new Double[3];
          for (int j = 0; j < 3; j++) {
            String xPosStr = lStringTok.nextToken();
            if (xPosStr.lastIndexOf(",") > 0) {
              xPosStr = xPosStr.substring(0, xPosStr.lastIndexOf(","));
            }
            val[j] = Double.parseDouble(xPosStr);
          }
          waypoints[i] = new Waypoint(val[0], val[1], val[2]);
        }

        tableData.Reset();
        UpdateUnits();

        System.out.println("Finished parsing properties file.");
      } catch (IOException e) {
        System.err.println("Could not open file connection!");
      } finally {
        try {
          mReader.close();
        } catch (Exception e) {
        }
      }
    }
  }

  public void SaveTrajectory() {
    fChooser = new JFileChooser();
    fChooser.setCurrentDirectory(new java.io.File(filePath));
    fChooser.setDialogTitle("Save Path Segment");
    fChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
    fChooser.setFileFilter(new FileNameExtensionFilter("CSV File", "csv"));

    //
    // disable the "All files" option.
    //
    fChooser.setAcceptAllFileFilterUsed(false);

    if (fChooser.showSaveDialog(frmPathEditor) == JFileChooser.APPROVE_OPTION) {
			/*
			 * System.out.println("getCurrentDirectory(): " +
			 * fChooser.getCurrentDirectory());
			 * System.out.println("getSelectedFile() : " +
			 * fChooser.getSelectedFile());
			 */

      File myFile;

      // Save the waypoints as a separate file so they can be modified
      // later
      String myFilename = fChooser.getSelectedFile().toString();
      myFilename = myFilename.replaceAll("\\.csv", "");

      BufferedWriter writer = null;
      try {
        // Open the text file to write to
        writer = new BufferedWriter(new FileWriter(myFilename + ".txt"));

        // Write out the preferences
        writer.write(config.fit.toString()); // Fit Method
        writer.newLine();
        writer.write(RobotModifier); // Robot Modifier
        writer.newLine();
        if (isRedSide) {
          writer.write("true"); // Red Alliance
          writer.newLine();
        } else {
          writer.write("false"); // Blue Alliance
          writer.newLine();
        }
        writer.write(selectedLengthUnit); // Length Unit
        writer.newLine();
        writer.write(selectedSpeedUnit); // Speed Unit
        writer.newLine();
        writer.write(selectedAngleUnit); // Angle Unit
        writer.newLine();

        writer.write(textVelocity.getText() + ", " + textAccel.getText() + ", " + textJerk
            .getText()); // Speed,Accel,Jerk
        writer.newLine();

        writer.write(textWidth.getText() + ", " + textLength.getText()); // Robot
        // Width,
        // Height
        writer.newLine();

        // This is the number of waypoints
        ArrayList<ArrayList<Double>> myData = tableData.getData();
        writer.write(Integer.toString(myData.size()));
        for (int i = 0; i < myData.size(); i++) {
          writer.newLine();
          writer.write(
              myData.get(i).get(1) + ", " + myData.get(i).get(2) + ", " + myData.get(i).get(3));
        }

        // Close the writer
        writer.close();
      } catch (Exception e) {
        e.printStackTrace();
      } finally {
        try {
          // Close the writer regardless of what happens...
          writer.close();
        } catch (Exception e) {
        }
      }

			/*
			 * // Regenerate the trajectories using the shifted to origin values
			 * Waypoint[] tempWaypoints = tableData.returnShiftedWaypoints();
			 * trajectory = Pathfinder.generate(tempWaypoints, config);
			 * GenerateDriveTrajectories(trajectory);
			 */

      // Save the .csv file with the generated paths

      switch (RobotModifier) {
        case "Basic":
          if (driveTrajectories[0] != null) {
            myFile = new File(myFilename + ".csv");
            Pathfinder.writeToCSV(myFile, driveTrajectories[0]);
          }
          break;
        case "Tank":
          if (driveTrajectories[0] != null) {
            myFile = new File(fChooser.getSelectedFile().toString() + "_Left.csv");
            Pathfinder.writeToCSV(myFile, driveTrajectories[0]);
          }

          if (driveTrajectories[1] != null) {
            myFile = new File(fChooser.getSelectedFile().toString() + "_Right.csv");
            Pathfinder.writeToCSV(myFile, driveTrajectories[1]);
          }

          break;
        case "Swerve":
          myFile = new File(fChooser.getSelectedFile().toString() + "_FrontLeft.csv");
          Pathfinder.writeToCSV(myFile, driveTrajectories[0]);

          if (driveTrajectories[1] != null) {
            myFile = new File(fChooser.getSelectedFile().toString() + "_FrontRight.csv");
            Pathfinder.writeToCSV(myFile, driveTrajectories[1]);
          }

          if (driveTrajectories[2] != null) {
            myFile = new File(fChooser.getSelectedFile().toString() + "_BackLeft.csv");
            Pathfinder.writeToCSV(myFile, driveTrajectories[2]);
          }

          if (driveTrajectories[3] != null) {
            myFile = new File(fChooser.getSelectedFile().toString() + "_BackRight.csv");
            Pathfinder.writeToCSV(myFile, driveTrajectories[3]);
          }
          break;
      }

    } else {
      System.out.println("No Selection ");
    }
  }

  public Double ConvertLength(Double val, String from, String to) {
    double g1 = 1.0;
    double g2 = 1.0;

    // Set the gain to turn the "From" units to meters
    switch (from) {
      case "in":
      case "inches":
        g1 = 0.0254;
        break;
      case "ft":
      case "feet":
        g1 = 0.3048;
        break;
      case "yd":
      case "yrd":
      case "yard":
      case "yards":
        g1 = 0.9144;
        break;
      case "m":
        g1 = 1.0;
        break;
    }

    // Set the gain to convert meters to the "to" units
    switch (to) {
      case "in":
      case "inches":
        g2 = 39.3701;
        break;
      case "ft":
      case "feet":
        g2 = 3.280841666667;
        break;
      case "yd":
      case "yrd":
      case "yard":
      case "yards":
        g2 = 1.0936138888889999077;
        break;
      case "m":
        g2 = 1.0;
        break;
    }

    // System.out.println("ConvertLength: " + val + " (" + from + ") -> " +
    // val*g1*g2 + " (" + to + ")");
    return val * g1 * g2;
  }

  public Double ConvertSpeed(Double val, String from, String to) {
    double g1 = 1.0;
    double g2 = 1.0;

    // Set the gain to turn the "From" units to meters/s
    switch (from) {
      case "fps":
      case "ft/s":
        g1 = 0.3048;
        break;
      case "mps":
      case "m/s":
        g1 = 1;
        break;
    }

    // Set the gain to convert meters/s to the "to" units
    switch (to) {
      case "fps":
      case "ft/s":
        g2 = 3.28084;
        break;
      case "mps":
      case "m/s":
        g2 = 1;
        break;
    }

    // System.out.println("ConvertSpeed: " + val + " (" + from + ") -> " +
    // val*g1*g2 + " (" + to + ")");
    return val * g1 * g2;
  }

  public Double ConvertAngle(Double val, String from, String to) {
    double g1 = 1.0;
    double g2 = 1.0;

    // Set the gain to turn the "From" units to rad
    switch (from) {
      case "deg":
      case "degrees":
        g1 = Math.PI / 180.0;
        break;
      case "rad":
      case "radians":
        g1 = 1;
        break;
    }

    // Set the gain to convert rad to the "to" units
    switch (to) {
      case "deg":
      case "degrees":
        g2 = 180.0 / Math.PI;
        break;
      case "rad":
      case "radians":
        g2 = 1;
        break;
    }

    // System.out.println("ConvertAngle: " + val + " (" + from + ") -> " +
    // val*g1*g2 + " (" + to + ")");
    return val * g1 * g2;
  }
}
