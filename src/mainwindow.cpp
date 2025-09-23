#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
  , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    /***********************
     * module initialization
     ***********************/
    CONFIG::instance(this);
    LOGGER::instance(this);
    UNIMAP::instance(this);
    OBSMAP::instance(this);
    MOBILE::instance(this);
    CAM::instance(this);
    LIDAR_2D::instance(this);
    LIDAR_3D::instance(this);
    LOCALIZATION::instance(this);
    MAPPING::instance(this);
    AUTOCONTROL::instance(this);
    SIM::instance(this);
    DOCKCONTROL::instance(this);
    POLICY::instance(this);

    COMM_COOP::instance(this);
    COMM_RRS::instance(this);
    COMM_MSA::instance(this);

    // for 3d viewer
    connect(ui->cb_ViewType,          SIGNAL(currentIndexChanged(QString)), this, SLOT(all_update()));   // change view type 2D, 3D, mapping -> update all rendering elements
    connect(ui->spb_PointSize,        SIGNAL(valueChanged(int)),            this, SLOT(map_update()));   // change point size -> update map rendering elements

    // for viewer control
    connect(ui->bt_ViewLeft,          &QPushButton::clicked, this, [this]()   { viewer_camera_relative_control(-2, 0, 0, 0, 0, 0);});
    connect(ui->bt_ViewRight,         &QPushButton::clicked, this, [this]()   { viewer_camera_relative_control(+2, 0, 0, 0, 0, 0);});
    connect(ui->bt_ViewUp,            &QPushButton::clicked, this, [this]()   { viewer_camera_relative_control(0, +2, 0, 0, 0, 0);});
    connect(ui->bt_ViewDown,          &QPushButton::clicked, this, [this]()   { viewer_camera_relative_control(0, -2, 0, 0, 0, 0);});
    connect(ui->bt_ViewZoomIn,        &QPushButton::clicked, this, [this]()   { viewer_camera_relative_control(0, 0, +2, 0, 0, 0);});
    connect(ui->bt_ViewZoomOut,       &QPushButton::clicked, this, [this]()   { viewer_camera_relative_control(0, 0, -2, 0, 0, 0);});

    // for simulation
    connect(ui->bt_SimInit,           SIGNAL(clicked()),  this, SLOT(bt_SimInit()));                     // simulation virtual localization initialization

    // config reload
    connect(ui->bt_ConfigLoad,        SIGNAL(clicked()),  this, SLOT(bt_ConfigLoad()));                  // reload config.json file

    // emergency stop
    connect(ui->bt_Emergency,         SIGNAL(clicked()),  this, SLOT(bt_Emergency()));                   // software emo button

    // mobile
    connect(ui->bt_Sync,              SIGNAL(clicked()),  this, SLOT(bt_Sync()));                        // manual sync (mobile, 2d lidar, 3d lidar)
    connect(ui->bt_MoveLinearX,       SIGNAL(clicked()),  this, SLOT(bt_MoveLinearX()));                 // move linear x only use odomety
    connect(ui->bt_MoveLinearY,       SIGNAL(clicked()),  this, SLOT(bt_MoveLinearY()));                 // move linear y only use odomety
    connect(ui->bt_MoveRotate,        SIGNAL(clicked()),  this, SLOT(bt_MoveRotate()));                  // move rotate only use odomety
    connect(ui->bt_MotorInit,         SIGNAL(clicked()),  this, SLOT(bt_MotorInit()));                   // manual motor power on

    // jog
    connect(ui->bt_JogF,              SIGNAL(pressed()),  this, SLOT(bt_JogF()));                        // if button pressed, move robot to front direction
    connect(ui->bt_JogB,              SIGNAL(pressed()),  this, SLOT(bt_JogB()));                        // if button pressed, move robot to back direction
    connect(ui->bt_JogL,              SIGNAL(pressed()),  this, SLOT(bt_JogL()));                        // if button pressed, move robot to left
    connect(ui->bt_JogR,              SIGNAL(pressed()),  this, SLOT(bt_JogR()));                        // if button pressed, move robot to right
    connect(ui->bt_JogF,              SIGNAL(released()), this, SLOT(bt_JogReleased()));                 // if button released, slow down and stop
    connect(ui->bt_JogB,              SIGNAL(released()), this, SLOT(bt_JogReleased()));                 // if button released, slow down and stop
    connect(ui->bt_JogL,              SIGNAL(released()), this, SLOT(bt_JogReleased()));                 // if button released, slow down and stop
    connect(ui->bt_JogR,              SIGNAL(released()), this, SLOT(bt_JogReleased()));                 // if button released, slow down and stop

    // mapping
    connect(ui->bt_MapBuild,          SIGNAL(clicked()),  this, SLOT(bt_MapBuild()));                    // mapping start
    connect(ui->bt_MapSave,           SIGNAL(clicked()),  this, SLOT(bt_MapSave()));                     // if mapping end, save map file
    connect(ui->bt_MapLoad,           SIGNAL(clicked()),  this, SLOT(bt_MapLoad()));                     // map load
    connect(ui->bt_MapLastLc,         SIGNAL(clicked()),  this, SLOT(bt_MapLastLc()));                   // manual loop closing
    connect(ui->bt_MapPause,          SIGNAL(clicked()),  this, SLOT(bt_MapPause()));
    connect(ui->bt_MapResume,         SIGNAL(clicked()),  this, SLOT(bt_MapResume()));

    // localization
    connect(ui->bt_LocInit,           SIGNAL(clicked()),  this, SLOT(bt_LocInit()));                     // specify the robot position to estimate the location
    connect(ui->bt_LocStart,          SIGNAL(clicked()),  this, SLOT(bt_LocStart()));                    // localization start
    connect(ui->bt_LocStop,           SIGNAL(clicked()),  this, SLOT(bt_LocStop()));                     // localization stop

    // obsmap
    connect(ui->bt_ObsClear,          SIGNAL(clicked()),     this, SLOT(bt_ObsClear()));                         // manual obstacle map clear
    connect(OBSMAP::instance(),       SIGNAL(obs_updated()), this, SLOT(obs_update()), Qt::QueuedConnection);    // if obsmap changed, plot update

    // autocontrol
    connect(ui->bt_AutoMove,          SIGNAL(clicked()),                    this, SLOT(bt_AutoMove()));                                      // move A to B (use stanley, differential type)
    connect(ui->bt_AutoMove2,         SIGNAL(clicked()),                    this, SLOT(bt_AutoMove2()));                                     // move A to B (use holonomic pure pursuit, mecanum type)
    connect(ui->bt_AutoMove3,         SIGNAL(clicked()),                    this, SLOT(bt_AutoMove3()));                                     // move A to B (use PID, turn & go mode)
    connect(ui->bt_AutoStop,          SIGNAL(clicked()),                    this, SLOT(bt_AutoStop()));                                      // stop auto-control
    connect(ui->bt_AutoPause,         SIGNAL(clicked()),                    this, SLOT(bt_AutoPause()));                                     // pause auto-control
    connect(ui->bt_AutoResume,        SIGNAL(clicked()),                    this, SLOT(bt_AutoResume()));                                    // resume auto-control
    connect(ui->bt_ReturnToCharging,  SIGNAL(clicked()),                    this, SLOT(bt_ReturnToCharging()));                              // move to assigned charging location
    connect(AUTOCONTROL::instance(),  SIGNAL(signal_local_path_updated()),  this, SLOT(slot_local_path_updated()),  Qt::QueuedConnection);   // if local path changed, plot update
    connect(AUTOCONTROL::instance(),  SIGNAL(signal_global_path_updated()), this, SLOT(slot_global_path_updated()), Qt::QueuedConnection);   // if global path changed, plot update

    // dockcontrol
    connect(ui->bt_DockStart,         SIGNAL(clicked()),                    this, SLOT(bt_DockStart()));
    connect(ui->bt_DockStop,          SIGNAL(clicked()),                    this, SLOT(bt_DockStop()));
    connect(ui->bt_UnDockStart,       SIGNAL(clicked()),                    this, SLOT(bt_UnDockStart()));

    // start docking
    connect(ui->ckb_PlotEnable,       SIGNAL(stateChanged(int)),            this, SLOT(vtk_viewer_update(int)));

    // for response
    connect(this,                     SIGNAL(signal_move_response(DATA_MOVE)),                 COMM_RRS::instance(), SLOT(send_move_response(DATA_MOVE)));
    connect(DOCKCONTROL::instance(),  SIGNAL(signal_dock_response(DATA_DOCK)),                 COMM_RRS::instance(), SLOT(send_dock_response(DATA_DOCK)));
    connect(AUTOCONTROL::instance(),  SIGNAL(signal_move_response(DATA_MOVE)),                 COMM_RRS::instance(), SLOT(send_move_response(DATA_MOVE)));
    connect(LOCALIZATION::instance(), SIGNAL(signal_localization_response(DATA_LOCALIZATION)), COMM_RRS::instance(), SLOT(send_localization_response(DATA_LOCALIZATION)));

    // annotation
    connect(ui->bt_DelNode,           SIGNAL(clicked()), this, SLOT(bt_DelNode()));
    connect(ui->bt_AnnotSave,         SIGNAL(clicked()), this, SLOT(bt_AnnotSave()));
    connect(ui->bt_QuickAddNode,      SIGNAL(clicked()), this, SLOT(bt_QuickAddNode()));

    // safety function
    connect(ui->bt_ClearMismatch,      SIGNAL(clicked()), this, SLOT(bt_ClearMismatch()));
    connect(ui->bt_ClearOverSpd,       SIGNAL(clicked()), this, SLOT(bt_ClearOverSpd()));
    connect(ui->bt_ClearObs,           SIGNAL(clicked()), this, SLOT(bt_ClearObs()));
    connect(ui->bt_ClearFieldMis,      SIGNAL(clicked()), this, SLOT(bt_ClearFieldMis()));
    connect(ui->bt_ClearInterlockStop, SIGNAL(clicked()), this, SLOT(bt_ClearInterlockStop()));
    connect(ui->bt_BumperStop,         SIGNAL(clicked()), this, SLOT(bt_BumperStop()));
    connect(ui->bt_Recover,            SIGNAL(clicked()), this, SLOT(bt_Recover()));
    connect(ui->bt_SetDetectModeT,     SIGNAL(clicked()), this, SLOT(bt_SetDetectModeT()));
    connect(ui->bt_SetDetectModeF,     SIGNAL(clicked()), this, SLOT(bt_SetDetectModeF()));
    connect(ui->bt_Request,            SIGNAL(clicked()), this, SLOT(bt_Request()));
    connect(ui->bt_LiftPowerOn,        SIGNAL(clicked()), this, SLOT(bt_LiftPowerOn()));
    connect(ui->bt_LiftPowerOff,       SIGNAL(clicked()), this, SLOT(bt_LiftPowerOff()));
    connect(ui->bt_SetLidarField,      SIGNAL(clicked()), this, SLOT(bt_SetLidarField()));

    // others
    connect(ui->bt_TestLed, SIGNAL(clicked()), this, SLOT(bt_TestLed()));
    connect(ui->ckb_PlotKfrm, SIGNAL(stateChanged(int)), this, SLOT(ckb_PlotKfrm()));

    // set effect
    init_ui_effect();

    // set plot window
    setup_vtk();

    // init modules
    init_modules();

    // plot timer (pcl-vtk viewr & Qlabel)
    plot_timer = new QTimer(this);
    connect(plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    plot_timer->start(50);
}

MainWindow::~MainWindow()
{
    plot_timer->stop();
    delete ui;
}

void MainWindow::map_update()
{
    is_map_update = true;
}

void MainWindow::topo_update()
{
    is_topo_update = true;
}

void MainWindow::pick_update()
{
    is_pick_update = true;
}

void MainWindow::obs_update()
{
    is_obs_update = true;
}

void MainWindow::all_update()
{
    pick = PICKING();

    map_update();
    topo_update();
    pick_update();
    obs_update();
}

void MainWindow::vtk_viewer_update(int val)
{
    if(val == 0)
    {
        all_plot_clear();
    }
    else
    {
        all_update();
    }
}

void MainWindow::set_opacity(QWidget* w, double opacity)
{
    auto* effect = new QGraphicsOpacityEffect(this);
    effect->setOpacity(opacity);
    w->setGraphicsEffect(effect);
}

void MainWindow::init_ui_effect()
{
    set_opacity(ui->lb_Screen1,     MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_Screen2,     MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_Screen3,     MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_Screen4,     MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_Screen5,     MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->bt_ViewLeft,    MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->bt_ViewRight,   MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->bt_ViewUp,      MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->bt_ViewDown,    MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->bt_ViewZoomIn,  MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->bt_ViewZoomOut, MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_PickPose,    MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_NodeId,      MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_RobotVel,    MAINWINDOW_INFO::opacity_val);
    set_opacity(ui->lb_RobotGoal,   MAINWINDOW_INFO::opacity_val);
}

void MainWindow::setup_vtk()
{
    // Set up the QVTK window
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);
    pcl_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    ui->qvtkWidget->setRenderWindow(pcl_viewer->getRenderWindow());
    pcl_viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
    pcl_viewer->setBackgroundColor(1.0, 1.0, 1.0);
    pcl_viewer->resetCamera();

    ui->qvtkWidget->grabGesture(Qt::PinchGesture);
    ui->qvtkWidget->installEventFilter(this);

    // init drawing
    pcl_viewer->addCoordinateSystem(1.0, "O_global");
    ui->qvtkWidget->renderWindow()->Render();

    // install event filter
    ui->qvtkWidget->installEventFilter(this);
    ui->qvtkWidget->setMouseTracking(true);
}

void MainWindow::init_modules()
{
    // load config
    if(CONFIG::instance()->load_common(QCoreApplication::applicationDirPath() + "/config/common.json"))
    {
        QString robot_type_str = CONFIG::instance()->get_robot_type_str();
        ui->lb_RobotType->setText(robot_type_str);

        mileage = CONFIG::instance()->get_mileage();

        if(robot_type_str.isEmpty())
        {
            QMessageBox::warning(this, "Config Load Failed", "Failed to load common config file.\nPlease check the path or configuration.");
        }
        else
        {
            QString path = QCoreApplication::applicationDirPath() + "/config/" + robot_type_str + "/config.json";
            CONFIG::instance()->set_config_path(path);
            CONFIG::instance()->load();

            QString path_version = QCoreApplication::applicationDirPath() + "/version.json";
            CONFIG::instance()->set_version_path(path_version);
            CONFIG::instance()->load_version();

            QString path_cam_serial_number = QCoreApplication::applicationDirPath() + "/config/" + robot_type_str + "/config_sn.json";
            CONFIG::instance()->set_serial_number_path(path_cam_serial_number);
            CONFIG::instance()->load_cam_serial_number();
        }
    }
    else
    {
        QMessageBox::warning(this, "Config Load Failed", "Failed to load common config file.\nPlease check the path or configuration.");
    }

    // simulation check
    if(CONFIG::instance()->get_use_sim())
    {
        this->setWindowTitle("SLAMNAV2 (SIMULATION MODE)");

        QPalette palette = this->palette();
        palette.setColor(QPalette::Window, Qt::red);
        this->setPalette(palette);

        ui->bt_SimInit->setEnabled(true);
    }

    // view mode
    if(CONFIG::instance()->get_loc_mode() == "3D")
    {
        ui->cb_ViewType->setCurrentText("VIEW_3D");
    }

    // log module init
    {
        LOGGER::instance()->set_log_path(QCoreApplication::applicationDirPath() + "/snlog/");
        LOGGER::instance()->init();
    }

    // unimap module init
    {
        UNIMAP::instance()->set_config_module(CONFIG::instance());
        UNIMAP::instance()->set_logger_module(LOGGER::instance());
    }

    // obsmap module init
    {
        OBSMAP::instance()->set_config_module(CONFIG::instance());
        OBSMAP::instance()->set_logger_module(LOGGER::instance());
        OBSMAP::instance()->set_unimap_module(UNIMAP::instance());
        OBSMAP::instance()->init();
    }

    // mobile module init
    {
        MOBILE::instance()->set_config_module(CONFIG::instance());
        MOBILE::instance()->set_logger_module(LOGGER::instance());
        MOBILE::instance()->open();
    }

    // cam module init
    {
        if(CONFIG::instance()->get_use_cam())
        {
            CAM::instance()->set_config_module(CONFIG::instance());
            CAM::instance()->set_logger_module(LOGGER::instance());
            CAM::instance()->set_mobile_module(MOBILE::instance());
            CAM::instance()->init();
            CAM::instance()->open();
        }
    }

    // lidar 2d module init
    {
        if(CONFIG::instance()->get_use_lidar_2d())
        {
            LIDAR_2D::instance()->set_config_module(CONFIG::instance());
            LIDAR_2D::instance()->set_logger_module(LOGGER::instance());
            LIDAR_2D::instance()->set_mobile_module(MOBILE::instance());
            LIDAR_2D::instance()->init();
            LIDAR_2D::instance()->open();
        }
    }

    // lidar 3d module init
    {
        if(CONFIG::instance()->get_use_lidar_3d())
        {
            LIDAR_3D::instance()->set_config_module(CONFIG::instance());
            LIDAR_3D::instance()->set_logger_module(LOGGER::instance());
            LIDAR_3D::instance()->init();
            LIDAR_3D::instance()->open();
        }
    }

    // localization module init
    {
        LOCALIZATION::instance()->set_config_module(CONFIG::instance());
        LOCALIZATION::instance()->set_logger_module(LOGGER::instance());
        LOCALIZATION::instance()->set_mobile_module(MOBILE::instance());
        LOCALIZATION::instance()->set_lidar_2d_module(LIDAR_2D::instance());
        LOCALIZATION::instance()->set_lidar_3d_module(LIDAR_3D::instance());
        LOCALIZATION::instance()->set_cam_module(CAM::instance());
        LOCALIZATION::instance()->set_unimap_module(UNIMAP::instance());
        LOCALIZATION::instance()->set_obsmap_module(OBSMAP::instance());
    }

    // mapping module init
    {
        if(CONFIG::instance()->get_use_lidar_2d())
        {
            MAPPING::instance()->set_config_module(CONFIG::instance());
            MAPPING::instance()->set_logger_module(LOGGER::instance());
            MAPPING::instance()->set_unimap_module(UNIMAP::instance());
            MAPPING::instance()->set_lidar_2d_module(LIDAR_2D::instance());
            MAPPING::instance()->set_localization_module(LOCALIZATION::instance());
        }
    }

    // autocontrol module init
    {
        AUTOCONTROL::instance()->set_config_module(CONFIG::instance());
        AUTOCONTROL::instance()->set_logger_module(LOGGER::instance());
        AUTOCONTROL::instance()->set_mobile_module(MOBILE::instance());
        // ctrl->lidar = lidar;
        // ctrl->cam = cam;
        AUTOCONTROL::instance()->set_localization_module(LOCALIZATION::instance());
        AUTOCONTROL::instance()->set_unimap_module(UNIMAP::instance());
        AUTOCONTROL::instance()->set_obsmap_module(OBSMAP::instance());
        AUTOCONTROL::instance()->init();
    }

    // simulation module init
    {
        SIM::instance()->set_config_module(CONFIG::instance());
        SIM::instance()->set_logger_module(LOGGER::instance());
        SIM::instance()->set_unimap_module(UNIMAP::instance());
        SIM::instance()->set_mobile_module(MOBILE::instance());
        if(CONFIG::instance()->get_loc_mode() == "3D")
        {
            SIM::instance()->set_lidar_3d_module(LIDAR_3D::instance());
        }
        else
        {
            SIM::instance()->set_lidar_2d_module(LIDAR_2D::instance());
        }
        SIM::instance()->set_localization_module(LOCALIZATION::instance());
    }

    // comm cooperative module init
    {
        COMM_COOP::instance()->set_config_module(CONFIG::instance());
        COMM_COOP::instance()->set_logger_module(LOGGER::instance());
        COMM_COOP::instance()->set_mobile_module(MOBILE::instance());
        COMM_COOP::instance()->set_unimap_module(UNIMAP::instance());
        COMM_COOP::instance()->set_obsmap_module(OBSMAP::instance());
        COMM_COOP::instance()->set_autocontrol_module(AUTOCONTROL::instance());
        COMM_COOP::instance()->set_localization_module(LOCALIZATION::instance());
    }

    // comm cooperative module init
    {
        COMM_RRS::instance()->set_config_module(CONFIG::instance());
        COMM_RRS::instance()->set_logger_module(LOGGER::instance());
        COMM_RRS::instance()->set_mobile_module(MOBILE::instance());
        COMM_RRS::instance()->set_unimap_module(UNIMAP::instance());
        COMM_RRS::instance()->set_obsmap_module(OBSMAP::instance());
        COMM_RRS::instance()->set_lidar_2d_module(LIDAR_2D::instance());
        COMM_RRS::instance()->set_autocontrol_module(AUTOCONTROL::instance());
        COMM_RRS::instance()->set_localization_module(LOCALIZATION::instance());
        COMM_RRS::instance()->set_mapping_module(MAPPING::instance());
        COMM_RRS::instance()->set_dockcontrol_module(DOCKCONTROL::instance());
        COMM_RRS::instance()->init();
    }

    //comm cooperative module init(msa)
    {
        COMM_MSA::instance()->set_config_module(CONFIG::instance());
        COMM_MSA::instance()->set_logger_module(LOGGER::instance());
        COMM_MSA::instance()->set_mobile_module(MOBILE::instance());
        COMM_MSA::instance()->set_unimap_module(UNIMAP::instance());
        COMM_MSA::instance()->set_obsmap_module(OBSMAP::instance());
        COMM_MSA::instance()->set_lidar_2d_module(LIDAR_2D::instance());
        COMM_MSA::instance()->set_autocontrol_module(AUTOCONTROL::instance());
        COMM_MSA::instance()->set_localization_module(LOCALIZATION::instance());
        COMM_MSA::instance()->set_mapping_module(MAPPING::instance());
        COMM_MSA::instance()->set_dockcontrol_module(DOCKCONTROL::instance());
        COMM_MSA::instance()->init();
    }

    // docking module init
    {
        DOCKCONTROL::instance()->set_config_module(CONFIG::instance());
        DOCKCONTROL::instance()->set_logger_module(LOGGER::instance());
        DOCKCONTROL::instance()->set_mobile_module(MOBILE::instance());
        DOCKCONTROL::instance()->set_lidar_2d_module(LIDAR_2D::instance());
        DOCKCONTROL::instance()->set_obsmap_module(OBSMAP::instance());
    }

    // zone module init
    {
        POLICY::instance()->set_config_module(CONFIG::instance());
        POLICY::instance()->set_logger_module(LOGGER::instance());
        POLICY::instance()->set_unimap_module(UNIMAP::instance());
        POLICY::instance()->set_localization_module(LOCALIZATION::instance());
        POLICY::instance()->init();
        POLICY::instance()->open();
    }

    // start jog loop
    jog_flag = true;
    jog_thread = std::make_unique<std::thread>(&MainWindow::jog_loop, this);

    // start watchdog loop
    watch_flag = true;
    watch_thread = std::make_unique<std::thread>(&MainWindow::watch_loop, this);

    // auto load
    QString map_path = CONFIG::instance()->get_map_path();
    if(!map_path.isEmpty())
    {
        UNIMAP::instance()->load_map(map_path);
    }
    all_update();
    bt_Request();
}

void MainWindow::all_plot_clear()
{
    // clear all registered info
    pcl_viewer->removeAllCoordinateSystems();
    pcl_viewer->removeAllPointClouds();
    pcl_viewer->removeAllShapes();

    // add global axis
    pcl_viewer->addCoordinateSystem(1.0, "O_global");
}

// for picking interface
bool MainWindow::eventFilter(QObject *object, QEvent *ev)
{
    if(object == ui->qvtkWidget && UNIMAP::instance()->get_is_loaded() == MAP_LOADED)
    {
        // touch to right mouse
        if(ui->ckb_ViewTouchToRightButton->isChecked())
        {
            if(ev->type() == QEvent::TouchBegin || ev->type() == QEvent::TouchUpdate || ev->type() == QEvent::TouchEnd)
            {
                auto touch_event = static_cast<QTouchEvent *>(ev);
                if(!touch_event->touchPoints().isEmpty())
                {
                    const QTouchEvent::TouchPoint &point = touch_event->touchPoints().first();
                    QPointF pos = point.pos(); // touch point position

                    QEvent::Type mouse_event_type;
                    if(ev->type() == QEvent::TouchBegin)
                    {
                        mouse_event_type = QEvent::MouseButtonPress;
                    }
                    else if(ev->type() == QEvent::TouchUpdate)
                    {
                        mouse_event_type = QEvent::MouseMove;
                    }
                    else if(ev->type() == QEvent::TouchEnd)
                    {
                        mouse_event_type = QEvent::MouseButtonRelease;
                    }

                    QMouseEvent *mouse_event = new QMouseEvent(mouse_event_type, pos, Qt::RightButton, Qt::RightButton, Qt::NoModifier);
                    QApplication::postEvent(ui->qvtkWidget, mouse_event);
                }
                return false;
            }
        }
        else
        {
            if(ev->type() == QEvent::TouchBegin || ev->type() == QEvent::TouchUpdate || ev->type() == QEvent::TouchEnd)
            {
                auto touch_event = static_cast<QTouchEvent *>(ev);
                if(!touch_event->touchPoints().isEmpty())
                {
                    const QTouchEvent::TouchPoint &point = touch_event->touchPoints().first();
                    QPointF pos = point.pos(); // touch point position

                    QEvent::Type mouse_event_type;
                    if(ev->type() == QEvent::TouchBegin)
                    {
                        mouse_event_type = QEvent::MouseButtonPress;
                    }
                    else if(ev->type() == QEvent::TouchUpdate)
                    {
                        mouse_event_type = QEvent::MouseMove;
                    }
                    else if(ev->type() == QEvent::TouchEnd)
                    {
                        mouse_event_type = QEvent::MouseButtonRelease;
                    }

                    QMouseEvent *mouse_event = new QMouseEvent(mouse_event_type, pos, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                    QApplication::postEvent(ui->qvtkWidget, mouse_event);
                }
                return false;
            }
        }

        if(ui->cb_ViewType->currentText() == "VIEW_2D")
        {
            // mouse event
            if(ev->type() == QEvent::MouseButtonPress)
            {
                QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                if(me->button() == Qt::LeftButton)
                {
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget->size().width();
                    double h = ui->qvtkWidget->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.cur_node = UNIMAP::instance()->get_goal_id(pt);

                    std::cout << "pick.cur_node:" << pick.cur_node.toStdString() << std::endl;

                    // update last mouse button
                    pick.last_btn = 0;

                    pick_update();
                    return true;
                }

                if(me->button() == Qt::RightButton)
                {
                    // clear node selection
                    pick.pre_node = "";
                    pick.cur_node = "";

                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget->size().width();
                    double h = ui->qvtkWidget->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt0 = pt;
                    pick.r_drag = true;

                    // calc pose
                    pick.r_pose[0] = pick.r_pt0[0];
                    pick.r_pose[1] = pick.r_pt0[1];
                    pick.r_pose[2] = 0;

                    // update last mouse button
                    pick.last_btn = 1;

                    pick_update();
                    return true;
                }
            }

            if(ev->type() == QEvent::MouseMove)
            {
                QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                if(pick.l_drag)
                {
                    pick_update();
                    return true;
                }

                if(pick.r_drag)
                {
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget->size().width();
                    double h = ui->qvtkWidget->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt1 = pt;

                    // calc pose
                    pick.r_pose[0] = pick.r_pt0[0];
                    pick.r_pose[1] = pick.r_pt0[1];
                    pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                    pick_update();
                    return true;
                }
            }

            if(ev->type() == QEvent::MouseButtonRelease)
            {
                QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                if(me->button() == Qt::LeftButton)
                {
                    pick_update();
                    return true;
                }

                if(me->button() == Qt::RightButton)
                {
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget->size().width();
                    double h = ui->qvtkWidget->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt1 = pt;
                    pick.r_drag = false;

                    // calc pose
                    pick.r_pose[0] = pick.r_pt0[0];
                    pick.r_pose[1] = pick.r_pt0[1];
                    pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                    pick_update();
                    return true;
                }
            }
        }

        // cam control
        if(ui->cb_ViewType->currentText() == "VIEW_3D")
        {
            if(Qt::ShiftModifier == QApplication::keyboardModifiers())
            {
                // mouse event
                if(ev->type() == QEvent::MouseButtonPress)
                {
                    QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                    if(me->button() == Qt::LeftButton)
                    {
                        // ray casting
                        double x = me->pos().x();
                        double y = me->pos().y();
                        double w = ui->qvtkWidget->size().width();
                        double h = ui->qvtkWidget->size().height();

                        Eigen::Vector3d ray_center;
                        Eigen::Vector3d ray_direction;
                        picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                        Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                        pick.cur_node = UNIMAP::instance()->get_goal_id(pt);

                        //                        std::cout << "pick.cur_node:" << pick.cur_node.toStdString() << std::endl;

                        // update last mouse button
                        pick.last_btn = 0;

                        pick_update();
                        return true;
                    }

                    if(me->button() == Qt::RightButton)
                    {
                        // clear node selection
                        pick.pre_node = "";
                        pick.cur_node = "";

                        // ray casting
                        double x = me->pos().x();
                        double y = me->pos().y();
                        double w = ui->qvtkWidget->size().width();
                        double h = ui->qvtkWidget->size().height();

                        Eigen::Vector3d ray_center;
                        Eigen::Vector3d ray_direction;
                        picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                        Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                        pick.r_pt0 = pt;
                        pick.r_drag = true;

                        // calc pose
                        pick.r_pose[0] = pick.r_pt0[0];
                        pick.r_pose[1] = pick.r_pt0[1];
                        pick.r_pose[2] = 0;

                        // update last mouse button
                        pick.last_btn = 1;

                        pick_update();
                        return true;
                    }
                }

                if(ev->type() == QEvent::MouseMove)
                {
                    QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                    if(pick.l_drag)
                    {
                        pick_update();
                        return true;
                    }

                    if(pick.r_drag)
                    {
                        // ray casting
                        double x = me->pos().x();
                        double y = me->pos().y();
                        double w = ui->qvtkWidget->size().width();
                        double h = ui->qvtkWidget->size().height();

                        Eigen::Vector3d ray_center;
                        Eigen::Vector3d ray_direction;
                        picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                        Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                        pick.r_pt1 = pt;

                        // calc pose
                        pick.r_pose[0] = pick.r_pt0[0];
                        pick.r_pose[1] = pick.r_pt0[1];
                        pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                        pick_update();
                        return true;
                    }
                }

                if(ev->type() == QEvent::MouseButtonRelease)
                {
                    QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                    if(me->button() == Qt::LeftButton)
                    {
                        pick_update();
                        return true;
                    }

                    if(me->button() == Qt::RightButton)
                    {
                        // ray casting
                        double x = me->pos().x();
                        double y = me->pos().y();
                        double w = ui->qvtkWidget->size().width();
                        double h = ui->qvtkWidget->size().height();

                        Eigen::Vector3d ray_center;
                        Eigen::Vector3d ray_direction;
                        picking_ray(x, y, w, h, ray_center, ray_direction, pcl_viewer);

                        Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                        pick.r_pt1 = pt;
                        pick.r_drag = false;

                        // calc pose
                        pick.r_pose[0] = pick.r_pt0[0];
                        pick.r_pose[1] = pick.r_pt0[1];
                        pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                        pick_update();
                        return true;
                    }
                }
            }
        }
    }

    return QWidget::eventFilter(object, ev);
}

void MainWindow::viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz)
{
    std::vector<pcl::visualization::Camera> cams;
    pcl_viewer->getCameras(cams);

    Eigen::Vector3d pos(cams[0].pos[0], cams[0].pos[1], cams[0].pos[2]);
    Eigen::Vector3d focal(cams[0].focal[0], cams[0].focal[1], cams[0].focal[2]);
    Eigen::Vector3d up(cams[0].view[0], cams[0].view[1], cams[0].view[2]);
    Eigen::Vector3d look = focal - pos;
    Eigen::Vector3d right = look.cross(up);

    double d = look.norm();

    look.normalize();
    right.normalize();
    up = right.cross(look);

    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf.block<3, 1>(0, 0) = right;
    tf.block<3, 1>(0, 1) = up;
    tf.block<3, 1>(0, 2) = look;
    tf.block<3, 1>(0, 3) = pos;

    Eigen::Matrix4d delta_tf = ZYX_to_TF(tx, ty, tz, rx, ry, rz);
    Eigen::Matrix4d tf_new = tf * ZYX_to_TF(0, 0, d, 0, 0, 0) * delta_tf * ZYX_to_TF(0, 0, -d, 0, 0, 0);

    Eigen::Vector3d up_new = tf_new.block<3, 1>(0, 1);
    Eigen::Vector3d pos_new = tf_new.block<3, 1>(0, 3);
    Eigen::Vector3d focal_new = tf_new.block<3, 1>(0, 2)*d + pos_new;

    pcl_viewer->setCameraPosition(pos_new[0], pos_new[1], pos_new[2],
            focal_new[0], focal_new[1], focal_new[2],
            up_new[0], up_new[1], up_new[2]);


    pcl_viewer->setCameraClipDistances(2.0, 1000.0);
}

void MainWindow::picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer)
{
    // ray casting
    std::vector<pcl::visualization::Camera> cams;
    pcl_viewer->getCameras(cams);

    Eigen::Matrix4d proj;
    Eigen::Matrix4d view;

    pcl::visualization::Camera cam0;
    try
    {
        cam0 = cams.at(0);
    }
    catch(std::out_of_range)
    {
        //logger.write_log("pcl viewer not have camera ..", "Red", true, false);
        center = Eigen::Vector3d(0,0,0);
        dir = Eigen::Vector3d(0,0,0);
        return;
    }

    cam0.computeProjectionMatrix(proj);
    cam0.computeViewMatrix(view);

    Eigen::Vector3d pos(cam0.pos[0], cam0.pos[1], cam0.pos[2]);

    Eigen::Vector4d p_ndc;
    p_ndc[0] = (2.0*u)/w - 1.0;
    p_ndc[1] = 1.0 - (2.0*v)/h;
    p_ndc[2] = -1.0;
    p_ndc[3] = 1.0;

    Eigen::Vector4d p_view = proj.inverse()*p_ndc;
    Eigen::Vector4d q_view = p_view;
    q_view[3] = 0;

    Eigen::Vector4d q_world = view.inverse()*q_view;
    Eigen::Vector3d ray(q_world[0], q_world[1], q_world[2]);
    ray.normalize();

    center = pos;
    dir = ray;
}

Eigen::Vector3d MainWindow::ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal)
{
    double d = plane_normal.dot(plane_center);
    double s = plane_normal.dot(ray_direction);
    if(std::abs(s) < 0.001)
    {
        return plane_center;
    }

    double t = (d-(plane_normal.dot(ray_center)))/s;
    return ray_center + t*ray_direction;
}

void MainWindow::update_jog_values(double vx, double vy, double wz)
{
    last_jog_update_time = get_time();

    vx_target = vx;
    vy_target = vy;
    wz_target = wz;
}

double MainWindow::apply_jog_acc(double cur_vel, double tgt_vel, double acc, double dcc, double dt)
{
    double err = tgt_vel - cur_vel;

    if(tgt_vel != 0)
    {
        cur_vel += saturation(err, -acc*dt, acc*dt);
    }
    else
    {
        cur_vel += saturation(err, -dcc*dt, dcc*dt);
    }

    return cur_vel;
}

// for mobile platform
void MainWindow::bt_SimInit()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        LOGGER::instance()->write_log("[SIM] map load first", "Red", true, false);
        return;
    }

    // loc stop
    LOCALIZATION::instance()->stop();

    // stop first
    SIM::instance()->stop();

    // set
    Eigen::Matrix4d tf = se2_to_TF(pick.r_pose);
    SIM::instance()->set_cur_tf(tf);
    LOCALIZATION::instance()->set_cur_tf(tf);

    // start
    SIM::instance()->start();

    // make dummy
    MOBILE::instance()->set_is_connected(true);
    MOBILE::instance()->set_is_synced(true);

    LIDAR_2D::instance()->set_is_connected(true);
    LIDAR_2D::instance()->set_sync_flag(true);

    // loc start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    LOCALIZATION::instance()->start();
}

void MainWindow::bt_ConfigLoad()
{
    CONFIG::instance()->load();
}

// for emergency stop
void MainWindow::bt_Emergency()
{
    // task.cancel();
    AUTOCONTROL::instance()->stop();
    AUTOCONTROL::instance()->set_multi_request("none");
    AUTOCONTROL::instance()->set_obs_condition("none");
    // dctrl.stop();
    MOBILE::instance()->move(0,0,0);
}

// mobile
void MainWindow::bt_MotorInit()
{
    MOBILE::instance()->motor_on();
}

void MainWindow::bt_Sync()
{
    MOBILE::instance()->sync();

    if(CONFIG::instance()->get_use_lidar_2d())
    {
        LIDAR_2D::instance()->set_sync_flag(true);
    }

    if(CONFIG::instance()->get_use_lidar_3d())
    {
        LIDAR_3D::instance()->set_is_sync(true);
    }
}

void MainWindow::bt_MoveLinearX()
{
    AUTOCONTROL::instance()->set_is_moving(true);
    QTimer::singleShot(1000, [&]()
    {
        double d = ui->dsb_MoveLinearDist->value();
        double v = ui->dsb_MoveLinearV->value();
        double t = std::abs(d/v) + 0.5;

        MOBILE::instance()->move_linear_x(d, v);

        QTimer::singleShot(t*1000, [&]()
        {
            AUTOCONTROL::instance()->set_is_moving(false);
        });
    });
}

void MainWindow::bt_MoveLinearY()
{
    AUTOCONTROL::instance()->set_is_moving(true);
    QTimer::singleShot(1000, [&]()
    {
        double d = ui->dsb_MoveLinearDist->value();
        double v = ui->dsb_MoveLinearV->value();
        double t = std::abs(d/v) + 0.5;

        MOBILE::instance()->move_linear_y(d, v);

        QTimer::singleShot(t*1000, [&]()
        {
            AUTOCONTROL::instance()->set_is_moving(false);
        });
    });
}

void MainWindow::bt_MoveRotate()
{
    AUTOCONTROL::instance()->set_is_moving(true);
    QTimer::singleShot(1000, [&]()
    {
        double th = ui->dsb_MoveRotateDeg->value() * D2R;
        double w = ui->dsb_MoveRotateW->value() * D2R;
        double t = std::abs(th/w) + 0.5;

        MOBILE::instance()->move_rotate(th, w);

        QTimer::singleShot(t*1000, [&]()
        {
            AUTOCONTROL::instance()->set_is_moving(false);
        });
    });
}

void MainWindow::bt_MoveStop()
{
    AUTOCONTROL::instance()->set_is_moving(false);
    MOBILE::instance()->stop();
}

void MainWindow::bt_JogF()
{
    is_jog_pressed.store(true);
    update_jog_values(ui->spb_JogV->value(), 0, 0);
}

void MainWindow::bt_JogB()
{
    is_jog_pressed.store(true);
    update_jog_values(-ui->spb_JogV->value(), 0, 0);
}

void MainWindow::bt_JogL()
{
    is_jog_pressed.store(true);
    update_jog_values(0, 0, ui->spb_JogW->value()*D2R);
}

void MainWindow::bt_JogR()
{
    is_jog_pressed.store(true);
    update_jog_values(0, 0, -ui->spb_JogW->value()*D2R);
}

void MainWindow::bt_JogReleased()
{
    is_jog_pressed.store(false);
    update_jog_values(0, 0, 0);
}

// mapping
void MainWindow::bt_MapBuild()
{
    if(CONFIG::instance()->get_use_sim())
    {
        LOGGER::instance()->write_log("[MAIN] map build not allowed SIM_MODE", "Red", true, false);
        return;
    }

    // stop first
    MAPPING::instance()->stop();

    // clear
    all_plot_clear();
    UNIMAP::instance()->clear();

    // mapping start
    MAPPING::instance()->start();
}

void MainWindow::bt_MapSave()
{
    if(CONFIG::instance()->get_use_sim())
    {
        LOGGER::instance()->write_log("[MAIN] map save not allowed SIM_MODE", "Red", true, false);
        return;
    }

    if(!change_map_name)
    {
        // auto generation dir path
        QString _map_dir = "/data/maps/" + get_time_str();
        printf("[MAIN] Creating map directory: %s\n", _map_dir.toLocal8Bit().data());

        QDir().mkpath(_map_dir);

        UNIMAP::instance()->set_map_path(_map_dir);
        map_dir = _map_dir;
    }
    else
    {
        QString _map_dir = "/data/maps/" + map_dir;
        printf("[MAIN] Using existing map directory: %s\n", _map_dir.toLocal8Bit().data());
        QDir().mkpath(_map_dir);

        UNIMAP::instance()->set_map_path(_map_dir);
        map_dir = _map_dir;
    }

    // check
    QString map_path = UNIMAP::instance()->get_map_path();
    if(map_path.isEmpty())
    {
        LOGGER::instance()->write_log("[MAIN] no map_dir", "Red", true, false);
        return;
    }

    // stop first
    MAPPING::instance()->stop();

    // check kfrm
    if(MAPPING::instance()->get_kfrm_storage_size() == 0)
    {
        printf("[MAIN] no keyframe\n");
        return;
    }

    // get all points and sampling
    const int64_t p1 = 73856093;
    const int64_t p2 = 19349669;
    const int64_t p3 = 83492791;
    const double voxel_size = CONFIG::instance()->get_mapping_voxel_size();
    const auto kfrm_storage = MAPPING::instance()->get_kfrm_storage();

    std::unordered_map<int64_t, uint8_t> hash_map;
    std::vector<PT_XYZR> pts;
    for(size_t p = 0; p < kfrm_storage->size(); p++)
    {
        Eigen::Matrix4d G = (*kfrm_storage)[p].opt_G;
        for(size_t q = 0; q < (*kfrm_storage)[p].pts.size(); q++)
        {
            Eigen::Vector3d P;
            P[0] = (*kfrm_storage)[p].pts[q].x;
            P[1] = (*kfrm_storage)[p].pts[q].y;
            P[2] = (*kfrm_storage)[p].pts[q].z;

            Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);

            // int64_t x = _P[0];
            // int64_t y = _P[1];
            // int64_t z = _P[2];

            int64_t x = std::floor(_P[0]/voxel_size);
            int64_t y = std::floor(_P[1]/voxel_size);
            int64_t z = std::floor(_P[2]/voxel_size);
            int64_t key = x*p1 ^ y*p2 ^ z*p3; // unlimited bucket size
            if(hash_map.find(key) == hash_map.end())
            {
                hash_map[key] = 1;

                PT_XYZR pt;
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                pt.r = (*kfrm_storage)[p].pts[q].r;
                pts.push_back(pt);
            }
        }

        printf("[MAIN] convert: %d/%d ..\n", (int)(p+1), (int)kfrm_storage->size());
    }

    // write file
    QString cloud_csv_path = map_path + "/cloud.csv";
    QFile cloud_csv_file(cloud_csv_path);
    if(cloud_csv_file.open(QIODevice::WriteOnly|QFile::Truncate))
    {
        for(size_t p = 0; p < pts.size(); p++)
        {
            double x = pts[p].x;
            double y = pts[p].y;
            double z = pts[p].z;
            double r = pts[p].r;

            QString str;
            str.sprintf("%f,%f,%f,%f\n", x, y, z, r);
            cloud_csv_file.write(str.toUtf8());
        }

        cloud_csv_file.close();
        printf("[MAIN] %s saved, pts: %zu\n", cloud_csv_path.toLocal8Bit().data(), pts.size());
    }
}

void MainWindow::bt_MapLoad()
{
    //QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", "/data/maps");
    if(!path.isNull())
    {
        // clear first
        LOCALIZATION::instance()->stop();
        OBSMAP::instance()->clear();

        CONFIG::instance()->set_map_path(path);
        UNIMAP::instance()->load_map(path);
        all_update();
    }
}

void MainWindow::bt_MapLastLc()
{
    MAPPING::instance()->last_loop_closing();
}

void MainWindow::bt_MapPause()
{
    MAPPING::instance()->is_pause.store(true);
    printf("[MAIN] mapping pasue\n");
}

void MainWindow::bt_MapResume()
{
    MAPPING::instance()->is_pause.store(false);
    printf("[MAIN] mapping resume\n");
}

// localization
void MainWindow::bt_LocInit()
{
    LOCALIZATION::instance()->set_cur_tf(se2_to_TF(pick.r_pose));
}

void MainWindow::bt_LocStart()
{
    LOCALIZATION::instance()->start();
}

void MainWindow::bt_LocStop()
{
    LOCALIZATION::instance()->stop();
}

// for autocontrol
void MainWindow::bt_AutoMove()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    if(pick.cur_node != "")
    {
        NODE* node = UNIMAP::instance()->get_node_by_id(pick.cur_node);
        if(node)
        {
            Eigen::Vector3d xi = TF_to_se2(node->tf);

            DATA_MOVE msg;
            msg.command = "goal";
            msg.method = "pp";
            msg.preset = 0;
            msg.goal_node_id = node->id;
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2,3);
            msg.tgt_pose_vec[3] = xi[2];

            Q_EMIT (AUTOCONTROL::instance()->signal_move(msg));
        }
        return;
    }

    if(pick.last_btn == 1)
    {
        Eigen::Matrix4d tf = se2_to_TF(pick.r_pose);
        Eigen::Vector3d xi = TF_to_se2(tf);

        DATA_MOVE msg;
        msg.tgt_pose_vec[0] = xi[0];
        msg.tgt_pose_vec[1] = xi[1];
        msg.tgt_pose_vec[2] = 0;
        msg.tgt_pose_vec[3] = xi[2];

        Q_EMIT (AUTOCONTROL::instance()->signal_move(msg));
        return;
    }
}

void MainWindow::bt_AutoMove2()
{
    if(UNIMAP::instance()->get_is_loaded() == MAP_NOT_LOADED)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    if(pick.cur_node != "")
    {
        NODE* node = UNIMAP::instance()->get_node_by_id(pick.cur_node);
        if(node != NULL)
        {
            Eigen::Vector3d xi = TF_to_se2(node->tf);

            DATA_MOVE msg;
            msg.command = "goal";
            msg.method = "hpp";
            msg.preset = 0;
            msg.goal_node_id = node->id;
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2,3);
            msg.tgt_pose_vec[3] = xi[2];

            Q_EMIT (AUTOCONTROL::instance()->signal_move(msg));
        }
        return;
    }

    if(pick.last_btn == 1)
    {
        Eigen::Matrix4d tf = se2_to_TF(pick.r_pose);
        Eigen::Vector3d xi = TF_to_se2(tf);

        DATA_MOVE msg;
        msg.tgt_pose_vec[0] = xi[0];
        msg.tgt_pose_vec[1] = xi[1];
        msg.tgt_pose_vec[2] = 0;
        msg.tgt_pose_vec[3] = xi[2];

        Q_EMIT (AUTOCONTROL::instance()->signal_move(msg));
        return;
    }
}

void MainWindow::bt_AutoMove3()
{
    // turn & go move
}

void MainWindow::bt_AutoStop()
{
    AUTOCONTROL::instance()->stop();
}

void MainWindow::bt_AutoPause()
{
    AUTOCONTROL::instance()->set_is_pause(true);
}

void MainWindow::bt_AutoResume()
{
    AUTOCONTROL::instance()->set_is_pause(false);
}

//docking
void MainWindow::bt_DockStart()
{
    // no use OSSD
    MOBILE::instance()->set_detect_mode(0.0);
    AUTOCONTROL::instance()->set_is_moving(true);
    DOCKCONTROL::instance()->move();
}

void MainWindow::bt_DockStop()
{
    DOCKCONTROL::instance()->stop();
    AUTOCONTROL::instance()->set_is_moving(false);

    // use OSSD
    MOBILE::instance()->set_detect_mode(1.0);
}

void MainWindow::bt_UnDockStart()
{
    // no use OSSD
    MOBILE::instance()->set_detect_mode(0.0);

    AUTOCONTROL::instance()->set_is_moving(true);
    DOCKCONTROL::instance()->undock();

    double t = std::abs(CONFIG::instance()->get_robot_size_x_max() / 0.05) + 1.0;
    QTimer::singleShot(t*1000, [&]()
    {
        AUTOCONTROL::instance()->set_is_moving(false);

        // use OSSD
        MOBILE::instance()->set_detect_mode(1.0);
        DOCKCONTROL::instance()->stop();
    });
}

//for safety
void MainWindow::bt_ClearMismatch()
{
    MOBILE::instance()->clearmismatch();
}

void MainWindow::bt_ClearOverSpd()
{
    MOBILE::instance()->clearoverspd();
}

void MainWindow::bt_ClearObs()
{
    MOBILE::instance()->clearobs();
}

void MainWindow::bt_ClearFieldMis()
{
    MOBILE::instance()->clearfieldmis();
}

void MainWindow::bt_ClearInterlockStop()
{
    MOBILE::instance()->clearinterlockstop();
}

void MainWindow::bt_BumperStop()
{
    MOBILE::instance()->clearbumperstop();
}

void MainWindow::bt_Recover()
{
    MOBILE::instance()->recover();
}

void MainWindow::bt_SetDetectModeT()
{
    MOBILE::instance()->set_detect_mode(1.0);
}

void MainWindow::bt_SetDetectModeF()
{
    MOBILE::instance()->set_detect_mode(0.0);
}

void MainWindow::bt_Request()
{
    MOBILE::instance()->robot_request();
}

void MainWindow::bt_LiftPowerOn()
{
    MOBILE::instance()->lift_power_onoff(1);
}

void MainWindow::bt_LiftPowerOff()
{
    MOBILE::instance()->lift_power_onoff(0);
}

void MainWindow::bt_SetLidarField()
{
    unsigned int field = ui->le_Lidar_Field_Write->text().toUInt();
    MOBILE::instance()->setlidarfield(field);
}

void MainWindow::bt_TestLed()
{
    MOBILE::instance()->led(0, ui->spb_Led->value());
    printf("[MAIN] led test:%d\n", ui->spb_Led->value());
}

void MainWindow::ckb_PlotKfrm()
{
    if(ui->ckb_PlotKfrm->isChecked())
    {
        int num = MAPPING::instance()->kfrm_storage->size();
        for(int p = 0; p < num; p++)
        {
            MAPPING::instance()->kfrm_update_que.push(p);
        }
    }
}

void MainWindow::bt_ReturnToCharging()
{
    QString robot_serial_number = CONFIG::instance()->get_robot_serial_number();
    if(robot_serial_number == "RB-M-")
    {
        LOGGER::instance()->write_log("[RTC] Robot serial number is not properly set.", "Red");
        return;
    }

    auto unimap_nodes = UNIMAP::instance()->get_nodes_origin();

    std::vector<QString> found_ids;
    for(const auto& node : (*unimap_nodes))
    {
        QString name = node.name;
        if(name.contains(robot_serial_number))
        {
            found_ids.push_back(node.id);
        }
    }

    if(found_ids.size() == 0)
    {
        LOGGER::instance()->write_log(QString("[RTC] No charging node found for robot serial: %1").arg(robot_serial_number), "Red");
        return;
    }

    if(found_ids.size() > 1)
    {
        LOGGER::instance()->write_log(QString("[RTC] Multiple charging nodes found for serial: %1 (count: %2)")
                                      .arg(robot_serial_number)
                                      .arg(found_ids.size()), "Red");
        return;
    }


    QString found_id = found_ids[0];
    NODE* node = UNIMAP::instance()->get_node_by_id(found_id);
    if(node == nullptr)
    {
        LOGGER::instance()->write_log(QString("[RTC] Failed to retrieve node with ID: %1").arg(found_id), "Red");
        return;
    }

    LOGGER::instance()->write_log(QString("[RTC] Found charging node: %1").arg(node->name), "Green");

    Eigen::Vector3d xi = TF_to_se2(node->tf);
    DATA_MOVE msg;
    msg.command = "goal";
    msg.method = "pp";
    msg.preset = 0;
    msg.goal_node_id = node->id;
    msg.tgt_pose_vec[0] = xi[0];
    msg.tgt_pose_vec[1] = xi[1];
    msg.tgt_pose_vec[2] = node->tf(2,3);
    msg.tgt_pose_vec[3] = xi[2];

    Q_EMIT (AUTOCONTROL::instance()->signal_move(msg));
}

// annotation
void MainWindow::bt_DelNode()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        printf("[Del_Node] check map load\n");
        return;
    }

    if(LOCALIZATION::instance()->get_is_loc() == false)
    {
        printf("[Del_Node] check localization\n");
        return;
    }

    if(pick.cur_node == "")
    {
        printf("[Del_Node] check pick node\n");
        return;
    }

    UNIMAP::instance()->remove_node(pick.cur_node);
    topo_update();
}

void MainWindow::bt_AnnotSave()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        printf("[Del_Node] check map load\n");
        return;
    }

    if(LOCALIZATION::instance()->get_is_loc() == false)
    {
        printf("[Del_Node] check localization\n");
        return;
    }

    UNIMAP::instance()->save_node();
}

void MainWindow::bt_QuickAddNode()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        printf("[QA_Node] check map load\n");
        return;
    }

    if(LOCALIZATION::instance()->get_is_loc() == false)
    {
        printf("[QA_Node] check localization\n");
        return;
    }

    Eigen::Matrix4d cur_tf = LOCALIZATION::instance()->get_cur_tf();
    UNIMAP::instance()->add_node(cur_tf, "GOAL");

    topo_update();
    printf("[QA_Node] quick add node\n");
}


void MainWindow::slot_local_path_updated()
{
    COMM_RRS::instance()->set_local_path_update();
    COMM_MSA::instance()->set_local_path_update();
    is_local_path_update = true;
}

void MainWindow::slot_global_path_updated()
{
    COMM_RRS::instance()->set_global_path_update();
    COMM_MSA::instance()->set_global_path_update();
    is_global_path_update = true;
}

// for obsmap
void MainWindow::bt_ObsClear()
{
    OBSMAP::instance()->clear();
    obs_update();
}

// jog
void MainWindow::jog_loop()
{
    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    printf("[JOG] loop start\n");
    while(jog_flag)
    {
        // check autodrive
        if(!AUTOCONTROL::instance()->get_is_moving() /*|| AUTOCONTROL::instance()->get_is_debug()*/)
        {
            // action
            double v_acc   = ui->spb_AccV->value();
            double v_decel = ui->spb_DecelV->value();
            double w_acc   = ui->spb_AccW->value()*D2R;
            double w_decel = ui->spb_DecelW->value()*D2R;

            Eigen::Vector3d control_input = MOBILE::instance()->get_control_input();
            double vx0 = control_input[0];
            double vy0 = control_input[1];
            double wz0 = control_input[2];

            double vx = apply_jog_acc(vx0, vx_target, v_acc, v_decel, dt);
            double vy = apply_jog_acc(vy0, vy_target, v_acc, v_decel, dt);
            double wz = apply_jog_acc(wz0, wz_target, w_acc, w_decel, dt);
            if(!is_jog_pressed && (get_time() - last_jog_update_time) > 1.0)
            {
                if(std::abs((double)vx_target) > 1e-09 || std::abs((double)vy_target) > 1e-09 || std::abs((double)wz_target) > 1e-09)
                {
                    vx_target = 0.;
                    vy_target = 0.;
                    wz_target = 0.;
                    printf("[JOG] no input, stop\n");
                }
            }
            MOBILE::instance()->move(vx, vy, wz);
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            printf("[JOG] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    printf("[JOG] loop stop\n");
}

// watchdog
void MainWindow::watch_loop()
{
    int cnt = 0;
    int speaker_cnt = 0;
    int loc_fail_cnt = 0;
    double last_sync_time = 0;

    bool is_first_emo_check = true;
    MOBILE_STATUS pre_ms = MOBILE::instance()->get_status();

    printf("[WATCHDOG] loop start\n");
    while(watch_flag)
    {
        cnt++;

        // for 500ms
        if(cnt % 2 == 0)
        {
            if(MOBILE::instance()->get_is_connected())
            {
                MOBILE_STATUS ms = MOBILE::instance()->get_status();
                if(is_first_emo_check)
                {
                    is_first_emo_check = false;
                    pre_ms = ms;
                }
                else
                {
                    if(ms.t != 0)
                    {
                        if(pre_ms.motor_stop_state == 0 && ms.motor_stop_state >= 1)
                        {
                            Eigen::Vector3d cur_pos = LOCALIZATION::instance()->get_cur_tf().block(0,3,3,1);

                            DATA_MOVE move_info;
                            move_info.cur_pos = cur_pos;
                            move_info.result = "emo";
                            move_info.message = "released";
                            move_info.time = get_time();

                            Q_EMIT signal_move_response(move_info);
                        }
                        else if(pre_ms.motor_stop_state >= 1 && ms.motor_stop_state == 0)
                        {
                            Eigen::Vector3d cur_pos = LOCALIZATION::instance()->get_cur_tf().block(0,3,3,1);

                            DATA_MOVE move_info;
                            move_info.cur_pos = cur_pos;
                            move_info.result = "emo";
                            move_info.message = "pushed";
                            move_info.time = get_time();

                            if(AUTOCONTROL::instance())
                            {
                                AUTOCONTROL::instance()->set_multi_inter_lock(true);
                            }

                            Q_EMIT signal_move_response(move_info);
                        }

                        pre_ms = ms;
                    }
                }
            }
        }

        // for 500ms loop
        if(cnt % 5 == 0)
        {
            //For sem docking retry logic
            bool retry_flag = DOCKCONTROL::instance()->get_dock_retry_flag();

            if(retry_flag)
            {
                DOCKCONTROL::instance()->stop();
                DOCKCONTROL::instance()->move();
                DOCKCONTROL::instance()->set_dock_retry_flag(false);
            }

            // Led logic
            int led_state = led_handler();
            MOBILE::instance()->led(0, led_state);

            // speaker logic
            if(CONFIG::instance()->get_robot_use_speaker() == true)
            {
                speaker_cnt++;
                speaker_handler(speaker_cnt);
            }

            // check mobile
            if(MOBILE::instance()->get_is_connected())
            {
                if(!MOBILE::instance()->get_is_synced())
                {
                    MOBILE::instance()->sync();
                    last_sync_time = get_time();
                    LOGGER::instance()->write_log("[WATCH] try time sync, pc and mobile");
                }

                MOBILE_STATUS ms = MOBILE::instance()->get_status();
                if(ms.t != 0)
                {
                    // when motor status 0, emo released, no charging
                    if((ms.status_m0 == 0 || ms.status_m1 == 0) && ms.motor_stop_state == 1 && ms.charge_state == 0)
                    {
                        //                        MOBILE::instance()->motor_on();
                    }

                    if(ms.connection_m0 == 1 && ms.connection_m1 == 1 &&
                            ms.status_m0 == 1 && ms.status_m1 == 1 &&
                            ms.motor_stop_state == 1 && ms.charge_state == 0)
                    {
                        MOBILE::instance()->set_cur_pdu_state("good");
                    }
                    else
                    {
                        MOBILE::instance()->set_cur_pdu_state("fail");
                        //                        led_color = LED_MAGENTA;
                    }
                }
            }

            // check lidar 3d
            if(CONFIG::instance()->get_use_lidar_3d())
            {
                if(LIDAR_3D::instance()->get_is_connected() && !LIDAR_3D::instance()->get_is_sync())
                {
                    LIDAR_3D::instance()->set_is_sync(true);
                    QString str = QString("[WATCH] try time sync, pc and lidar, type: %1").arg(CONFIG::instance()->get_lidar_3d_type());
                    printf("%s\n", str.toLocal8Bit().data());
                }
            }

            // check lidar 2d
            if(CONFIG::instance()->get_use_lidar_2d())
            {
                if(LIDAR_2D::instance()->get_is_connected() && !LIDAR_2D::instance()->get_is_sync())
                {
                    LIDAR_2D::instance()->set_sync_flag(true);
                    QString str = QString("[WATCH] try time sync, pc and lidar, type: %1").arg(CONFIG::instance()->get_lidar_2d_type());
                    printf("%s\n", str.toLocal8Bit().data());
                }
            }

            // resync for time skew
            if(get_time() - last_sync_time > 1200.0 && !AUTOCONTROL::instance()->get_is_moving())
            {
                // printf("[WATCH] resync all\n");
                LOGGER::instance()->write_log("[WATCH] resync all");

                if(MOBILE::instance()->get_is_connected())
                {
                    MOBILE::instance()->sync();
                }

                if(LIDAR_2D::instance()->get_is_connected())
                {
                    LIDAR_2D::instance()->set_sync_flag(true);
                }

                if(LIDAR_3D::instance()->get_is_connected())
                {
                    LIDAR_3D::instance()->set_is_sync(true);
                }

                last_sync_time = get_time();
            }

            // check loc
            if(CONFIG::instance()->get_use_sim())
            {
                LOCALIZATION::instance()->set_cur_loc_state("good");
            }
            else
            {
                if(!LOCALIZATION::instance()->get_is_loc())
                {
                    loc_fail_cnt = 0;
                    LOCALIZATION::instance()->set_cur_loc_state("none");
                    //                    led_color = LED_MAGENTA;
                }
                else
                {
                    Eigen::Vector2d ieir = LOCALIZATION::instance()->get_cur_ieir();
                    if(ieir[0] > CONFIG::instance()->get_loc_2d_check_inlier_ratio() ||
                            ieir[1] < CONFIG::instance()->get_loc_2d_check_inlier_error())
                    {
                        loc_fail_cnt++;
                        if(loc_fail_cnt > 3)
                        {
                            LOCALIZATION::instance()->set_cur_loc_state("fail");
                            //                            led_color = LED_MAGENTA_BLINK;
                        }
                    }
                    else
                    {
                        loc_fail_cnt = 0;
                        LOCALIZATION::instance()->set_cur_loc_state("good");
                    }
                }
            }
        }

        // plot mobile distance info
        {
            double move_distance = prev_move_distance;
            double candidate = CONFIG::instance()->get_use_sim() ? SIM::instance()->distance : MOBILE::instance()->get_move_distance();

            if (!std::isnan(candidate))
            {
                move_distance = candidate;
            }
            float total_mileage = mileage + abs(move_distance);
            mileage_sum = QString::number(total_mileage, 'f', 3);
            prev_move_distance = move_distance;

            // plot mobile pose
            ui->lb_Mileage->setText("[Mileage] : "+mileage_sum);

            if(cnt % 100 == 0)
            {
                CONFIG::instance()->set_mileage(mileage_sum);
            }
        }


        // Samsung's request
        //        // for 500ms loop
        //        // obs logging
        //        if(cnt % 5 == 0)
        //        {
        //            if(AUTOCONTROL::instance()->get_is_moving())
        //            {
        //                double obs_d = AUTOCONTROL::instance()->get_obs_dist();

        //                if(obs_d < 2.0 + 1e-6)
        //                {
        //                    // zone, obs dist
        ////                    QString log = QString("\t%1\t%2")
        ////                            .arg(int(obs_d))
        ////                            .arg(obs_d, 0, 'f', 6);
        //                    QString log = QString().sprintf("\t%d\t%.3f", int(obs_d), obs_d);
        ////                    qDebug()<<log;

        //                    LOGGER::instance()->write_log_to_txt(log);
        //                }
        //            }
        //        }
        //        // for 1 min loop
        //        // temperature logging
        //        if(cnt % 600 == 0)
        //        {
        //            //for temperature status
        //            {
        //                MOBILE_STATUS mobile_log = MOBILE::instance()-> get_status();

        //                //            //m1 m2 battery usb sensor
        //                QString log = QString("\t%1\t%2\t%3\t%4\t%5")
        //                        .arg(mobile_log.temp_m0)
        //                        .arg(mobile_log.temp_m1)
        //                        .arg(mobile_log.tabos_temperature)
        //                        .arg(QString::number(temperature_value))
        //                        .arg(mobile_log.tabos_soc, 0, 'f', 3);

        //                //            qDebug()<<"QString::number(temperature_value) : "<<QString::number(temperature_value);
        //                //            qDebug()<<temperature_value.load();

        //                LOGGER::instance()->write_temperature_log_to_txt(log);
        //            }
        //        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[WATCHDOG] loop stop\n");
}

// for plot loop
void MainWindow::plot_map()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        return;
    }

    if(is_map_update)
    {
        is_map_update = false;

        if(ui->cb_ViewType->currentText() == "VIEW_2D")
        {
            auto kdtree_cloud = UNIMAP::instance()->get_kdtree_cloud();
            if(kdtree_cloud)
            {
                const int sampling_size = 4;

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                cloud->reserve(kdtree_cloud->pts.size() / sampling_size);
                for(size_t p = 0; p < kdtree_cloud->pts.size(); p += sampling_size) // sampling
                {

                    pcl::PointXYZRGB pt;
                    pt.x = kdtree_cloud->pts[p].x;
                    pt.y = kdtree_cloud->pts[p].y;
                    pt.z = kdtree_cloud->pts[p].z;

                    double reflect = std::sqrt(kdtree_cloud->pts[p].r/255);
                    tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                    pt.r = c.r()*255;
                    pt.g = c.g()*255;
                    pt.b = c.b()*255;

                    cloud->push_back(pt);
                }

                if(!pcl_viewer->updatePointCloud(cloud, "map_pts"))
                {
                    pcl_viewer->addPointCloud(cloud, "map_pts");
                }

                // point size
                pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size, "map_pts");
            }
        }
        else
        {
            if(pcl_viewer->contains("map_pts"))
            {
                pcl_viewer->removePointCloud("map_pts");
            }
        }

        // 3D
        if(ui->cb_ViewType->currentText() == "VIEW_3D")
        {
            auto pts = UNIMAP::instance()->get_map_3d_pts();
            auto reflects = UNIMAP::instance()->get_map_3d_reflects();
            if(pts && reflects)
            {
                const int point_size = pts->size();

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                cloud->reserve(point_size);
                for(size_t p = 0; p < point_size; p++)
                {
                    Eigen::Vector3d P = (*pts)[p];
                    double reflect = std::sqrt((*reflects)[p]/255);
                    tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                    pcl::PointXYZRGB pt;
                    pt.x = P[0];
                    pt.y = P[1];
                    pt.z = P[2];
                    pt.r = c.r()*255;
                    pt.g = c.g()*255;
                    pt.b = c.b()*255;
                    cloud->push_back(pt);
                }
                if(!pcl_viewer->updatePointCloud(cloud, "map_3d_pts"))
                {
                    pcl_viewer->addPointCloud(cloud, "map_3d_pts");
                }

                // point size
                pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->spb_PointSize->value(), "map_3d_pts");
            }
        }
        else
        {
            if(pcl_viewer->contains("map_3d_pts"))
            {
                pcl_viewer->removePointCloud("map_3d_pts");
            }
        }
    }
}

void MainWindow::plot_node()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        return;
    }

    // draw topo
    if(is_topo_update)
    {
        is_topo_update = false;

        // erase first
        if(last_plot_nodes.size() > 0)
        {
            for(size_t p = 0; p < last_plot_nodes.size(); p++)
            {
                QString id = last_plot_nodes[p];
                if(pcl_viewer->contains(id.toStdString()))
                {
                    pcl_viewer->removeShape(id.toStdString());
                }
            }
            last_plot_nodes.clear();
        }

        // remove names
        if(last_plot_names.size() > 0)
        {
            for(size_t p = 0; p < last_plot_names.size(); p++)
            {
                QString id = last_plot_names[p];
                if(pcl_viewer->contains(id.toStdString()))
                {
                    pcl_viewer->removeShape(id.toStdString());
                }
            }
            last_plot_names.clear();
        }

        // draw
        auto unimap_nodes = UNIMAP::instance()->get_nodes_origin();
        if(unimap_nodes && unimap_nodes->size() > 0)
        {
            // std::cout << "1" << std::endl;
            if(ui->ckb_PlotNodes->isChecked())
            {
                double x_min = CONFIG::instance()->get_robot_size_x_min(); double x_max = CONFIG::instance()->get_robot_size_x_max();
                double y_min = CONFIG::instance()->get_robot_size_y_min(); double y_max = CONFIG::instance()->get_robot_size_y_max();
                double z_min = CONFIG::instance()->get_robot_size_z_min(); double z_max = CONFIG::instance()->get_robot_size_z_max();

                // draw nodes
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

                const size_t nodes_size = unimap_nodes->size();
                cloud->reserve(nodes_size);
                for(const auto& node : (*unimap_nodes))
                {
                    QString id = node.id;
                    if(pcl_viewer->contains(id.toStdString()))
                    {
                        printf("duplicate: %s\n", id.toStdString().data());
                    }

                    Eigen::Matrix4d tf = node.tf;
                    Eigen::Matrix3d R = tf.block(0,0,3,3);
                    Eigen::Vector3d t = tf.block(0,3,3,1);

                    if(node.type == "ROUTE")
                    {
                        const double size = 0.1;
                        pcl_viewer->addCube(-size, size, -size, size, 0, size, 0.8, 0.8, 0.8, id.toStdString());
                        pcl_viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(tf.cast<float>()));

                        Eigen::Vector3d front_dot = R * Eigen::Vector3d(size - 0.05, 0, 0) + t;
                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + size;
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(node.type == "INIT")
                    {
                        pcl_viewer->addCube(x_min, x_max,
                                            y_min, y_max,
                                            z_min, z_max, 0.0, 1.0, 1.0, id.toStdString());
                        pcl_viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(tf.cast<float>()));

                        Eigen::Vector3d front_dot = R * Eigen::Vector3d(x_max - 0.05, 0, 0) + t;
                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + z_max;
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(node.type == "GOAL")
                    {
                        pcl_viewer->addCube(x_min, x_max,
                                            y_min, y_max,
                                            z_min, z_max, 0.5, 1.0, 0.0, id.toStdString());
                        pcl_viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(tf.cast<float>()));

                        Eigen::Vector3d front_dot = R*Eigen::Vector3d(x_max - 0.05, 0, 0) + t;
                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + z_max;
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(node.type == "STATION")
                    {
                        pcl_viewer->addCube(x_min, x_max,
                                            y_min, y_max,
                                            z_min, z_max, 0.5, 1.0, 0.5, id.toStdString());
                        pcl_viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(tf.cast<float>()));

                        Eigen::Vector3d front_dot = R * Eigen::Vector3d(x_max - 0.05, 0, 0) + t;
                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + z_max;
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(node.type == "OBS")
                    {
                        QString info = node.info;

                        NODE_INFO res;
                        if(parse_info(info, "SIZE", res))
                        {
                            pcl_viewer->addCube(-res.sz[0]/2, res.sz[0]/2,
                                    -res.sz[1]/2, res.sz[1]/2,
                                    -res.sz[2]/2, res.sz[2]/2, 1.0, 0.0, 1.0, id.toStdString());

                            pcl_viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(tf.cast<float>()));
                            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, id.toStdString());
                        }
                    }
                    else if(node.type == "ZONE")
                    {
                        QString info = node.info;
                        NODE_INFO res;
                        if(parse_info(info, "SIZE", res))
                        {
                            pcl_viewer->addCube(-res.sz[0]/2, res.sz[0]/2,
                                    -res.sz[1]/2, res.sz[1]/2,
                                    -res.sz[2]/2, res.sz[2]/2, 0.5, 1.0, 0.0, id.toStdString());

                            pcl_viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(tf.cast<float>()));
                            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, id.toStdString());
                        }
                    }
                    else if(node.type == "ARUCO")
                    {
                        const double size = 0.15;
                        pcl_viewer->addCube(-size, size, -size, size, -size, size, 0.0, 1.0, 1.0, id.toStdString());
                        pcl_viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(tf.cast<float>()));

                        Eigen::Vector3d front_dot = R * Eigen::Vector3d(size - 0.05, 0, 0) + t;
                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + size;
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }

                    // for erase
                    last_plot_nodes.push_back(id);
                }

                if(!pcl_viewer->updatePointCloud(cloud, "front_dots"))
                {
                    pcl_viewer->addPointCloud(cloud, "front_dots");
                }
                pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size*2, "front_dots");
            }

            if(ui->ckb_PlotEdges->isChecked())
            {
                auto unimap_nodes = UNIMAP::instance()->get_nodes_origin();

                // draw edges
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(const auto node0 : (*unimap_nodes))
                {
                    for(const auto node1_id : node0.linked)
                    {
                        NODE* node1 = UNIMAP::instance()->get_node_by_id(node1_id);
                        if(node1 == nullptr)
                        {
                            continue;
                        }

                        Eigen::Vector3d P0 = node0.tf.block(0,3,3,1);
                        Eigen::Vector3d P1 = node1->tf.block(0,3,3,1);
                        Eigen::Vector3d P_mid = (P0+P1)/2;

                        std::vector<Eigen::Vector3d> pts0 = sampling_line(P0, P_mid, 0.05);
                        for(size_t i = 0; i < pts0.size(); i++)
                        {
                            pcl::PointXYZRGB pt;
                            pt.x = pts0[i][0];
                            pt.y = pts0[i][1];
                            pt.z = pts0[i][2];
                            pt.r = 255;
                            pt.g = 0;
                            pt.b = 0;

                            cloud->push_back(pt);
                        }

                        std::vector<Eigen::Vector3d> pts1 = sampling_line(P1, P_mid, 0.05);
                        for(size_t i = 0; i < pts1.size(); i++)
                        {
                            pcl::PointXYZRGB pt;
                            pt.x = pts1[i][0];
                            pt.y = pts1[i][1];
                            pt.z = pts1[i][2];
                            pt.r = 0;
                            pt.g = 255;
                            pt.b = 0;

                            cloud->push_back(pt);
                        }

                        pcl::PointXYZRGB pt;
                        pt.x = P_mid[0];
                        pt.y = P_mid[1];
                        pt.z = P_mid[2] + 0.05;
                        pt.r = 0;
                        pt.g = 0;
                        pt.b = 255;

                        cloud2->push_back(pt);
                    }
                }

                if(!pcl_viewer->updatePointCloud(cloud, "edge_dots"))
                {
                    pcl_viewer->addPointCloud(cloud, "edge_dots");
                }
                pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size*2, "edge_dots");
                pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "edge_dots");

                if(!pcl_viewer->updatePointCloud(cloud2, "edge_chk_dots"))
                {
                    pcl_viewer->addPointCloud(cloud2, "edge_chk_dots");
                }
                pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size*2.5, "edge_chk_dots");
            }
        }
    }
}

void MainWindow::plot_pick()
{
    // draw cursors
    if(is_pick_update)
    {
        is_pick_update = false;

        // erase first
        if(pcl_viewer->contains("O_pick"))
        {
            pcl_viewer->removeCoordinateSystem("O_pick");
        }

        if(pcl_viewer->contains("pick_body"))
        {
            pcl_viewer->removeShape("pick_body");
        }

        if(pcl_viewer->contains("sel_cur"))
        {
            pcl_viewer->removeShape("sel_cur");
        }

        if(pick.cur_node != "")
        {
            NODE *node = UNIMAP::instance()->get_node_by_id(pick.cur_node);
            if(node != nullptr)
            {
                pcl::PolygonMesh donut = make_donut(CONFIG::instance()->get_robot_radius(), 0.05, node->tf, 1.0, 1.0, 1.0);
                pcl_viewer->addPolygonMesh(donut, "sel_cur");

                // plot text
                QString text = "id: " + node->id + ", name: " + node->name;
                ui->lb_NodeId->setText(text);
            }
        }

        if(pick.last_btn == 1)
        {
            Eigen::Matrix4d pick_tf = se2_to_TF(pick.r_pose);

            double x_min = CONFIG::instance()->get_robot_size_x_min(); double x_max = CONFIG::instance()->get_robot_size_x_max();
            double y_min = CONFIG::instance()->get_robot_size_y_min(); double y_max = CONFIG::instance()->get_robot_size_y_max();
            double z_min = CONFIG::instance()->get_robot_size_z_min(); double z_max = CONFIG::instance()->get_robot_size_z_max();

            // draw pose
            pcl_viewer->addCoordinateSystem(1.0, "O_pick");
            pcl_viewer->addCube(x_min, x_max,
                                y_min, y_max,
                                z_min, z_max, 0.5, 0.5, 0.5, "pick_body");
            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");

            pcl_viewer->updateShapePose("pick_body", Eigen::Affine3f(pick_tf.cast<float>()));
            pcl_viewer->updateCoordinateSystemPose("O_pick", Eigen::Affine3f(pick_tf.cast<float>()));

            // plot text
            QString text;
            text.sprintf("x:%.2f, y:%.2f, th:%.2f", pick.r_pose[0], pick.r_pose[1], pick.r_pose[2]*R2D);
            ui->lb_PickPose->setText(text);
        }
    }
}

void MainWindow::plot_info()
{
    // plot mobile info
    {
        double tabos_battery_soc = MOBILE::instance()->get_battery_soc();
        ui->lb_MobileStatusInfo->setText(MOBILE::instance()->get_status_text());

        if(MOBILE::instance()->get_is_connected())
        {
            // plot mobile pose
            ui->lb_MobilePoseInfo->setText(MOBILE::instance()->get_pose_text());

            // plot mobile status
            ui->lb_MobileStatusInfo->setText(MOBILE::instance()->get_status_text());
            ui->lb_Battery->setText("[BAT]" + QString::number(tabos_battery_soc)+"%");
        }
    }

    // plot slam info
    {
        QString text;

        // mapping
        if(MAPPING::instance()->get_is_mapping())
        {
            text = MAPPING::instance()->get_info_text();
        }

        // localization
        if(LOCALIZATION::instance()->get_is_loc())
        {
            text += LOCALIZATION::instance()->get_info_text();
        }

        ui->lb_SlamInfo->setText(text);
    }

    // 2d lidar info
    if(CONFIG::instance()->get_use_lidar_2d())
    {
        ui->lb_LidarInfo_2D->setText(LIDAR_2D::instance()->get_info_text());
    }

    // 3d lidar info
    if(CONFIG::instance()->get_use_lidar_3d())
    {
        ui->lb_LidarInfo_3D->setText(LIDAR_3D::instance()->get_info_text());
    }

    // plot auto info
    {
        QString _multi_state = "";
        int fsm_state = AUTOCONTROL::instance()->get_fsm_state();

//        qDebug()<<"print : "<<AUTOCONTROL::instance()->get_obs_dist();
        QString auto_info_str;
        auto_info_str.sprintf("[AUTO_INFO]\nfsm_state: %s\nis_moving: %s, is_pause: %s, obs: %s (%.3f),\nis_multi: %d, request: %s, multi_state: %s",
                              AUTO_FSM_STATE_STR[fsm_state].toLocal8Bit().data(),
                              AUTOCONTROL::instance()->get_is_moving() ? "1" : "0",
                              AUTOCONTROL::instance()->get_is_pause() ? "1" : "0",
                              AUTOCONTROL::instance()->get_obs_condition().toLocal8Bit().data(),
                              AUTOCONTROL::instance()->get_obs_dist(),
                              CONFIG::instance()->get_use_multi(),
                              AUTOCONTROL::instance()->get_multi_reqest_state().toLocal8Bit().data(),
                              _multi_state.toLocal8Bit().data());
        ui->lb_AutoInfo->setText(auto_info_str);
    }

}

void MainWindow::plot_safety()
{
    //for safety parameter plot
    MOBILE_SETTING cur_setting = MOBILE::instance()->get_setting();

    ui->le_Setting_Version->setText(QString().sprintf("%d", cur_setting.version));
    ui->le_Setting_Type->setText(QString().sprintf("%d", cur_setting.robot_type));

    ui->le_Setting_Limit_V->setText(QString().sprintf("%.2f",cur_setting.v_limit/1000.0));
    ui->le_Setting_Limit_V_Jog->setText(QString().sprintf("%.2f",cur_setting.v_limit_jog/1000.0));
    ui->le_Setting_Limit_W->setText(QString().sprintf("%.2f", cur_setting.w_limit));
    ui->le_Setting_Limit_W_Jog->setText(QString().sprintf("%.2f",cur_setting.w_limit_jog));
    ui->le_Setting_Limit_A->setText(QString().sprintf("%.2f", cur_setting.a_limit/1000.0));
    ui->le_Setting_Limit_A_Jog->setText(QString().sprintf("%.2f", cur_setting.a_limit_jog/1000.0));
    ui->le_Setting_Limit_B->setText(QString().sprintf("%.2f", cur_setting.b_limit));
    ui->le_Setting_Limit_B_Jog->setText(QString().sprintf("%.2f", cur_setting.b_limit_jog));

    ui->le_Setting_W_R->setText(QString().sprintf("%.2f", cur_setting.w_r));
    ui->le_Setting_W_S->setText(QString().sprintf("%.2f", cur_setting.w_s));
    ui->le_Setting_Gear->setText(QString().sprintf("%.2f", cur_setting.gear));
    ui->le_Setting_Dir->setText(QString().sprintf("%.2f", cur_setting.dir));

    ui->le_Setting_Limit_V_Monitor->setText(QString().sprintf("%.2f",cur_setting.v_limit_monitor/1000.0));
    ui->le_Setting_Limit_W_Monitor->setText(QString().sprintf("%.2f",cur_setting.w_limit_monitor));

    MOBILE_STATUS cur_status = MOBILE::instance()->get_status();

    QString state = "";
    if(cur_status.om_state == MOBILE_POWER_OFF)
    {
        state = "POWER_OFF";
    }
    else if(cur_status.om_state == MOBILE_MAIN_POWER_UP)
    {
        state = "MAIN_POWER_UP";
    }
    else if(cur_status.om_state == MOBILE_PC_POWER_UP)
    {
        state = "PC_POWER_UP";
    }
    else if(cur_status.om_state == MOBILE_ROBOT_POWER_OFF)
    {
        state = "ROBOT_POWER_OFF";
    }
    else if(cur_status.om_state == MOBILE_ROBOT_INITIALIZE)
    {
        state = "ROBOT_INITIALIZE";
    }
    else if(cur_status.om_state == MOBILE_NORMAL_OP)
    {
        state = "NORMAL_OP";
    }
    else if(cur_status.om_state == MOBILE_NORMAL_OP_AUTO)
    {
        state = "NORMAL_OP_AUTO";
    }
    else if(cur_status.om_state == MOBILE_NORMAL_OP_MANUAL)
    {
        state = "NORMAL_OP_MANUAL";
    }
    else if(cur_status.om_state == MOBILE_NORMAL_LOW_BAT)
    {
        state = "NORMAL_LOW_BAT";
    }
    else if(cur_status.om_state == MOBILE_OPERATIONL_STOP)
    {
        state = "OPERATIONL_STOP";
    }
    else if(cur_status.om_state == MOBILE_CHARGING)
    {
        state = "CHARGING";
    }
    else if(cur_status.om_state == MOBILE_CONFIGURATION)
    {
        state = "CONFIGURATION";
    }
    else
    {
        state = "";
    }

    ui->le_Op_Mode->setText(state);

    QString ri_state = "";
    if(cur_status.ri_state == MOBILE_RI_IDLE)
    {
        ri_state = "RI_IDLE";
    }
    else if(cur_status.ri_state == MOBILE_RI_SAFETY_CHECK)
    {
        ri_state = "RI_SAFETY_CHECK";
    }
    else if(cur_status.ri_state == MOBILE_RI_POWER_ON)
    {
        ri_state = "RI_POWER_ON";
    }
    else if(cur_status.ri_state == MOBILE_RI_POWER_CHECK)
    {
        ri_state = "RI_POWER_CHECK";
    }
    else if(cur_status.ri_state == MOBILE_RI_MOTOR_INIT)
    {
        ri_state = "RI_MOTOR_INIT";
    }
    else if(cur_status.ri_state == MOBILE_RI_MOTOR_CHECK)
    {
        ri_state = "RI_MOTOR_CHECK";
    }
    else if(cur_status.ri_state == MOBILE_RI_DONE)
    {
        ri_state = "RI_DONE";
    }
    else if(cur_status.ri_state == MOBILE_RI_FAIL)
    {
        ri_state = "RI_FAIL";
    }
    else
    {
        ri_state = "";
    }

    ui->le_Robot_Init_State->setText(ri_state);
    ui->le_Lidar_Field_Read->setText(QString().sprintf("%d", cur_status.lidar_field));

    //emo status
    if(cur_status.safety_state_emo_pressed_1 || cur_status.safety_state_emo_pressed_2)
    {
        ui->le_Safety_Emo_Pressed->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Emo_Pressed->setText(QString().sprintf("none"));
    }

    //speed mismatch
    if(cur_status.safety_state_ref_meas_mismatch_1 || cur_status.safety_state_ref_meas_mismatch_2)
    {
        ui->le_Safety_Ref_Meas_Mismatch->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Ref_Meas_Mismatch->setText(QString().sprintf("none"));
    }

    //overspeed
    if(cur_status.safety_state_over_speed_1 || cur_status.safety_state_over_speed_2)
    {
        ui->le_Safety_Over_Speed->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Over_Speed->setText(QString().sprintf("none"));
    }

    //obstacle detect
    if(cur_status.safety_state_obstacle_detected_1 || cur_status.safety_state_obstacle_detected_2)
    {
        ui->le_Safety_Obstacle_Detect->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Obstacle_Detect->setText(QString().sprintf("none"));
    }

    //speed field mismatch
    if(cur_status.safety_state_speed_field_mismatch_1 || cur_status.safety_state_speed_field_mismatch_1)
    {
        ui->le_Safety_Speed_Field_Mismatch->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Speed_Field_Mismatch->setText(QString().sprintf("none"));
    }

    //interlock detect
    if(cur_status.safety_state_interlock_stop_1 || cur_status.safety_state_interlock_stop_2)
    {
        ui->le_Safety_Interlock_Stop->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Interlock_Stop->setText(QString().sprintf("none"));
    }

    //bumper detect
    if(cur_status.safety_state_bumper_stop_1 || cur_status.safety_state_bumper_stop_2)
    {
        ui->le_Safety_Bumper_Stop->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Bumper_Stop->setText(QString().sprintf("none"));
    }

    //operation stop detect
    if(cur_status.operational_stop_state_flag_1 || cur_status.operational_stop_state_flag_2)
    {
        ui->le_Safety_Op_Stop_State->setText(QString().sprintf("trig"));
    }
    else
    {
        ui->le_Safety_Op_Stop_State->setText(QString().sprintf("none"));
    }
}

void MainWindow::plot_raw_2d()
{
    if(LIDAR_2D::instance()->get_is_connected() && ui->cb_ViewType->currentText() == "VIEW_2D" &&
            !MAPPING::instance()->get_is_mapping() && !LOCALIZATION::instance()->get_is_loc())
    {
        Eigen::Matrix4d cur_tf = LOCALIZATION::instance()->get_cur_tf();
        Eigen::Matrix3d cur_R = cur_tf.block(0,0,3,3);
        Eigen::Vector3d cur_t = cur_tf.block(0,3,3,1);

        for(int idx = 0; idx < CONFIG::instance()->get_lidar_2d_num(); idx++)
        {
            RAW_FRAME cur_frm = LIDAR_2D::instance()->get_cur_raw(idx);

            const size_t point_size = cur_frm.pts.size();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            cloud->reserve(point_size);
            for(size_t p = 0; p < point_size; p++)
            {
                Eigen::Vector3d P;
                P[0] = cur_frm.pts[p][0];
                P[1] = cur_frm.pts[p][1];
                P[2] = cur_frm.pts[p][2];
                Eigen::Vector3d _P = cur_R * P + cur_t;

                pcl::PointXYZRGB pt;
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                if(idx == 0)
                {
                    pt.r = 0; pt.g = 127; pt.b = 255;
                }
                else
                {
                    pt.r = 255; pt.g = 55; pt.b = 0;
                }

                cloud->push_back(pt);
            }

            // for debug docking clust
            std::vector<Eigen::Vector3d> cluster_point;
            cluster_point = DOCKCONTROL::instance()->get_cur_clust();

            for (const auto& pt : cluster_point)
            {
                Eigen::Vector3d _P = cur_R * pt + cur_t;

                pcl::PointXYZRGB pcl_pt;
                pcl_pt.x = _P.x();
                pcl_pt.y = _P.y();
                pcl_pt.r = 0;
                pcl_pt.g = 255;
                pcl_pt.b = 0;

                cloud->push_back(pcl_pt);
            }

            QString cloud_id = QString("lidar_2d_cur_pts_%1").arg(idx);
            if(!pcl_viewer->updatePointCloud(cloud, cloud_id.toStdString()))
            {
                pcl_viewer->addPointCloud(cloud, cloud_id.toStdString());
            }
            pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id.toStdString());
            pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, cloud_id.toStdString());
        }
    }
    else
    {
        // remove existing 2d lidar cloud
        for(int idx = 0; idx < CONFIG::instance()->get_lidar_2d_num(); idx++)
        {
            QString cloud_id = QString("lidar_2d_cur_pts_%1").arg(idx);
            std::string id = cloud_id.toStdString();
            if(pcl_viewer->contains(id))
            {
                pcl_viewer->removePointCloud(id);
            }
        }
    }
}

void MainWindow::plot_raw_3d()
{
    // plot 3d lidar
    if(LIDAR_3D::instance()->get_is_connected() && ui->cb_ViewType->currentText() == "VIEW_3D" &&
            !MAPPING::instance()->get_is_mapping() && !LOCALIZATION::instance()->get_is_loc())
    {
        Eigen::Matrix4d cur_tf = LOCALIZATION::instance()->get_cur_tf();
        Eigen::Matrix3d cur_R = cur_tf.block(0,0,3,3);
        Eigen::Vector3d cur_t = cur_tf.block(0,3,3,1);

        for(int idx = 0; idx < CONFIG::instance()->get_lidar_3d_num(); idx++)
        {
            // plot current lidar data
            LVX_FRM cur_frm = LIDAR_3D::instance()->get_cur_raw(idx);

            const size_t point_size = cur_frm.pts.size();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            cloud->reserve(point_size);
            for(size_t p = 0; p < point_size; p++)
            {
                Eigen::Vector3d P;
                P[0] = cur_frm.pts[p].x;
                P[1] = cur_frm.pts[p].y;
                P[2] = cur_frm.pts[p].z;
                Eigen::Vector3d _P = cur_R * P + cur_t;

                pcl::PointXYZRGB pt;
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                if(idx == 0)
                {
                    pt.r = 255; pt.g = 127; pt.b = 0;    // Orange for lidar 0
                }
                else
                {
                    pt.r =   0; pt.g = 200; pt.b = 255;  // Cyan for lidar 1
                }

                cloud->push_back(pt);
            }

            QString cloud_id = QString("lidar_3d_cur_pts_%1").arg(idx);
            if(!pcl_viewer->updatePointCloud(cloud, cloud_id.toStdString()))
            {
                pcl_viewer->addPointCloud(cloud, cloud_id.toStdString());
            }
            pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id.toStdString());
            pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, cloud_id.toStdString());

        }
    }
    else
    {
        // remove 3d lidar cloud
        for(int idx = 0; idx < CONFIG::instance()->get_lidar_3d_num(); idx++)
        {
            QString cloud_id = QString("lidar_3d_cur_pts_%1").arg(idx);
            std::string id = cloud_id.toStdString();

            if(pcl_viewer->contains(id))
            {
                pcl_viewer->removePointCloud(id);
            }
        }
    }
}

void MainWindow::plot_process_time()
{
    QString ctrl_time_str;
    {
        if(AUTOCONTROL::instance())
        {
            double ctrl_obs_time = AUTOCONTROL::instance()->get_process_time_obs() * 1000;
            double ctrl_node_time = AUTOCONTROL::instance()->get_process_time_node() * 1000;
            double ctrl_control_time = AUTOCONTROL::instance()->get_process_time_control() * 1000;

            ctrl_time_str = QString::asprintf("[AUTO] control:%.3f, obs:%.3f, node:%.3f\n", ctrl_control_time, ctrl_obs_time, ctrl_node_time);
        }
    }

    QString cam_time_str;
    {
        if(CAM::instance())
        {
            double cam_post_time = CAM::instance()->get_process_time_post(0) * 1000;
            cam_time_str = QString::asprintf("[CAM] post:%.3f\n", cam_post_time);
        }
    }

    QString lidar_2d_time_str;
    {
        if(LIDAR_2D::instance())
        {
            double lidar_2d_deskewing_time[2] = {0.0, 0.0};
            for(size_t p = 0; p < CONFIG::instance()->get_lidar_2d_num(); p++)
            {
                lidar_2d_deskewing_time[p] = LIDAR_2D::instance()->get_process_time_deskewing(p) * 1000;
            }
            double lidar_2d_merge_time = LIDAR_2D::instance()->get_process_time_merge() * 1000;

            lidar_2d_time_str = QString::asprintf("[LIDAR_2D] dsk(0,1):%.3f,%.3f, merge:%.3f\n",
                                                  lidar_2d_deskewing_time[0], lidar_2d_deskewing_time[1], lidar_2d_merge_time);
        }
    }

    QString lidar_3d_time_str;
    {
        if(LIDAR_3D::instance())
        {
            double lidar_3d_deskewing_time[2] = {0.0, 0.0};
            for(size_t p = 0; p < CONFIG::instance()->get_lidar_3d_num(); p++)
            {
                lidar_3d_deskewing_time[p] = LIDAR_3D::instance()->get_process_time_deskewing(p) * 1000;
            }
            double lidar_3d_merge_time = LIDAR_3D::instance()->get_process_time_merge() * 1000;

            lidar_3d_time_str = QString::asprintf("[LIDAR_3D] dsk(0,1):%.3f,%.3f, merge:%.3f\n",
                                                  lidar_3d_deskewing_time[0], lidar_3d_deskewing_time[1], lidar_3d_merge_time);
        }
    }

    QString loc_time_str;
    {
        if(LOCALIZATION::instance())
        {
            double loc_localizaion_time = LOCALIZATION::instance()->get_process_time_localization() * 1000;
            double loc_odometry_time = LOCALIZATION::instance()->get_process_time_odometry() * 1000;
            double loc_obs_time = LOCALIZATION::instance()->get_process_time_obs() * 1000;

            loc_time_str = QString::asprintf("[LOC] loc:%.3f, odo:%.3f\n\tobs:%.3f\n",
                                             loc_localizaion_time, loc_odometry_time, loc_obs_time);
        }
    }

    QString mobile_time_str;
    {
        if(MOBILE::instance())
        {
            double mobile_time = MOBILE::instance()->get_process_time_mobile() * 1000;
            mobile_time_str = QString::asprintf("[MOBILE] mobile:%.3f\n", mobile_time);
        }
    }

    QString obs_time_str;
    {
        if(OBSMAP::instance())
        {
            double obs_obs_update_time = OBSMAP::instance()->get_last_obs_update_time() * 1000;
            double obs_vobs_update_time = OBSMAP::instance()->get_last_vobs_update_time() * 1000;

            obs_time_str = QString::asprintf("[OBS] obs:%.3f, vobs:%.3f\n", obs_obs_update_time, obs_vobs_update_time);
        }
    }

    QString process_str = "[PROC]\n" + ctrl_time_str + cam_time_str + lidar_2d_time_str + lidar_3d_time_str + loc_time_str + mobile_time_str + obs_time_str;
    ui->lb_ProcTimeInfo->setText(process_str);
}

void MainWindow::plot_mapping()
{
    if(MAPPING::instance()->get_is_mapping())
    {
        // plot live cloud
        if(ui->ckb_PlotLive->isChecked())
        {
            const double icp_do_accum_num = CONFIG::instance()->get_mapping_icp_do_accum_num();
            auto live_cloud_pts = MAPPING::instance()->get_live_cloud_pts();
            if(live_cloud_pts)
            {
                const size_t point_size = live_cloud_pts->size();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                cloud->reserve(point_size);
                for(size_t p = 0; p < point_size; p++)
                {
                    // set pos
                    PT_XYZR P = (*live_cloud_pts)[p];
                    pcl::PointXYZRGB pt;
                    pt.x = P.x;
                    pt.y = P.y;
                    pt.z = P.z;

                    // set color
                    double reflect = std::sqrt(P.r/255);
                    tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                    pt.r = c.r()*255;
                    pt.g = c.g()*255;
                    pt.b = c.b()*255;

                    // dynamic object
                    if(P.do_cnt < icp_do_accum_num)
                    {
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;
                    }

                    cloud->push_back(pt);
                }

                if(!pcl_viewer->updatePointCloud(cloud, "live_tree_pts"))
                {
                    pcl_viewer->addPointCloud(cloud, "live_tree_pts");
                }

                // point size
                pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size, "live_tree_pts");
            }
        }
        else
        {
            // remove realtime slam pts
            if(pcl_viewer->contains("live_tree_pts"))
            {
                pcl_viewer->removePointCloud("live_tree_pts");
            }
        }

        // plot keyframe pts
        if(ui->ckb_PlotKfrm->isChecked())
        {
            int kfrm_id;
            if(MAPPING::instance()->try_pop_kfrm_update_que(kfrm_id))
            {
                Eigen::Matrix4d opt_G = Eigen::Matrix4d::Identity();
                KFRAME kfrm = MAPPING::instance()->get_kfrm(kfrm_id);
                opt_G = kfrm.opt_G;

                QString name;
                name.sprintf("kfrm_%d", kfrm_id);

                if(!pcl_viewer->contains(name.toStdString()))
                {
                    const size_t point_size = kfrm.pts.size();

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    cloud->reserve(point_size);
                    for(const auto& pts: kfrm.pts)
                    {
                        // no drawing dynamic object
                        if(pts.do_cnt < CONFIG::instance()->get_mapping_icp_do_accum_num())
                        {
                            continue;
                        }

                        // set pos
                        pcl::PointXYZRGB pt;
                        pt.x = pts.x;
                        pt.y = pts.y;
                        pt.z = pts.z;

                        // set color
                        double reflect = std::sqrt(pts.r/255);
                        tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                        pt.r = c.r()*255;
                        pt.g = c.g()*255;
                        pt.b = c.b()*255;

                        cloud->push_back(pt);
                    }

                    if(!pcl_viewer->updatePointCloud(cloud, name.toStdString()))
                    {
                        pcl_viewer->addPointCloud(cloud, name.toStdString());
                        pcl_viewer->updatePointCloudPose(name.toStdString(), Eigen::Affine3f(opt_G.cast<float>()));
                        last_plot_kfrms.push_back(name);
                    }

                    // point size
                    pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size, name.toStdString());
                }
                else
                {
                    pcl_viewer->updatePointCloudPose(name.toStdString(), Eigen::Affine3f(opt_G.cast<float>()));

                    // point size
                    pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size, name.toStdString());
                }
            }
        }
        else
        {
            // remove keyframes
            if(last_plot_kfrms.size() > 0)
            {
                for(size_t p = 0; p < last_plot_kfrms.size(); p++)
                {
                    QString name = last_plot_kfrms[p];
                    if(pcl_viewer->contains(name.toStdString()))
                    {
                        pcl_viewer->removeShape(name.toStdString());
                    }
                }
            }
        }
    }
    else
    {
        // remove first
        for(size_t p = 0; p < last_plot_kfrms.size(); p++)
        {
            QString id = last_plot_kfrms[p];
            if(pcl_viewer->contains(id.toStdString()))
            {
                pcl_viewer->removeShape(id.toStdString());
            }
        }
        last_plot_kfrms.clear();

        if(pcl_viewer->contains("live_tree_pts"))
        {
            pcl_viewer->removePointCloud("live_tree_pts");
        }
    }
}

void MainWindow::plot_loc()
{
    std::vector<Eigen::Vector3d> plot_cur_pts = LOCALIZATION::instance()->get_cur_global_scan();
    if(ui->cb_ViewType->currentText() == "VIEW_3D" && plot_cur_pts.size() > 0 && LOCALIZATION::instance()->get_is_loc() && !MAPPING::instance()->get_is_mapping())
    {
        // remove first
        for(size_t p = 0; p < last_plot_kfrms.size(); p++)
        {
            QString id = last_plot_kfrms[p];
            if(pcl_viewer->contains(id.toStdString()))
            {
                pcl_viewer->removeShape(id.toStdString());
            }
        }
        last_plot_kfrms.clear();

        if(pcl_viewer->contains("live_tree_pts"))
        {
            pcl_viewer->removePointCloud("live_tree_pts");
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        const size_t point_size = plot_cur_pts.size();
        cloud->reserve(point_size);
        for(const auto& P: plot_cur_pts)
        {
            pcl::PointXYZRGB pt;
            pt.x = P[0];
            pt.y = P[1];
            pt.z = P[2];
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            cloud->push_back(pt);
        }

        if(!pcl_viewer->updatePointCloud(cloud, "plot_cur_pts"))
        {
            pcl_viewer->addPointCloud(cloud, "plot_cur_pts");
        }

        pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plot_cur_pts");
        pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "plot_cur_pts");
    }
    else if(ui->cb_ViewType->currentText() == "VIEW_2D" && plot_cur_pts.size() > 0 && LOCALIZATION::instance()->get_is_loc() && !MAPPING::instance()->get_is_mapping())
    {
        // remove first
        for(size_t p = 0; p < last_plot_kfrms.size(); p++)
        {
            QString id = last_plot_kfrms[p];
            if(pcl_viewer->contains(id.toStdString()))
            {
                pcl_viewer->removeShape(id.toStdString());
            }
        }
        last_plot_kfrms.clear();

        if(pcl_viewer->contains("live_tree_pts"))
        {
            pcl_viewer->removePointCloud("live_tree_pts");
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        const size_t point_size = plot_cur_pts.size();
        cloud->reserve(point_size);
        for(const auto& P: plot_cur_pts)
        {
            pcl::PointXYZRGB pt;
            pt.x = P[0];
            pt.y = P[1];
            pt.z = P[2];
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            cloud->push_back(pt);
        }

        if(!pcl_viewer->updatePointCloud(cloud, "plot_cur_pts"))
        {
            pcl_viewer->addPointCloud(cloud, "plot_cur_pts");
        }

        pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plot_cur_pts");
        pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "plot_cur_pts");
    }
    else
    {
        if(pcl_viewer->contains("plot_cur_pts"))
        {
            pcl_viewer->removePointCloud("plot_cur_pts");
        }
    }
}

void MainWindow::plot_obs()
{
    if(UNIMAP::instance()->get_is_loaded() != MAP_LOADED)
    {
        return;
    }

    if(is_obs_update)
    {
        is_obs_update = false;

        Eigen::Matrix4d obs_tf = OBSMAP::instance()->get_map_tf();
        cv::Mat obs_map = OBSMAP::instance()->get_obs_map();
        cv::Mat dyn_map = OBSMAP::instance()->get_dyn_map();
        cv::Mat vir_map = OBSMAP::instance()->get_vir_map();

        if((obs_map.rows == 0 || obs_map.cols == 0) ||
                (dyn_map.rows == 0 || dyn_map.cols == 0) ||
                (vir_map.rows == 0 || vir_map.cols == 0))
        {
            return;
        }

        if(obs_map.rows != dyn_map.rows || dyn_map.rows != vir_map.rows || vir_map.rows != obs_map.rows)
        {
            return;
        }

        if(obs_map.cols != dyn_map.cols || dyn_map.cols != vir_map.cols || vir_map.cols != obs_map.cols)
        {
            return;
        }

        // plot grid map
        {
            cv::Mat plot_obs_map(obs_map.rows, obs_map.cols, CV_8UC3, cv::Scalar(0));
            OBSMAP::instance()->draw_robot(plot_obs_map);

            for(int i = 0; i < plot_obs_map.rows; i++)
            {
                for(int j = 0; j < plot_obs_map.cols; j++)
                {
                    if(obs_map.ptr<uchar>(i)[j] == 255)
                    {
                        plot_obs_map.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(255,255,255);
                    }

                    if(dyn_map.ptr<uchar>(i)[j] == 255)
                    {
                        plot_obs_map.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(0,0,255);
                    }

                    if(vir_map.ptr<uchar>(i)[j] == 255)
                    {
                        plot_obs_map.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(0,255,255);
                    }
                }
            }

            ui->lb_Screen1->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot_obs_map)));
            ui->lb_Screen1->setScaledContents(true);
            ui->lb_Screen1->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

            // debug
            // cv::namedWindow("OBS MAP", cv::WINDOW_NORMAL);
            // cv::resizeWindow("OBS MAP", 700, 700);
            // cv::imshow("OBS MAP", plot_obs_map);
        }

        // plot obs pts
        std::vector<Eigen::Vector4d> plot_pts = OBSMAP::instance()->get_plot_pts();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < plot_pts.size(); p+=4)
        {
            double x = plot_pts[p][0];
            double y = plot_pts[p][1];
            double z = plot_pts[p][2];
            double prob = plot_pts[p][3];

            if(prob <= 0.5)
            {
                continue;
            }

            tinycolormap::Color c = tinycolormap::GetColor(prob, tinycolormap::ColormapType::Jet);

            pcl::PointXYZRGB pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;
            pt.r = c.r()*255;
            pt.g = c.g()*255;
            pt.b = c.b()*255;
            cloud->push_back(pt);
        }

        if(!pcl_viewer->updatePointCloud(cloud, "obs_plot_pts"))
        {
            pcl_viewer->addPointCloud(cloud, "obs_plot_pts");
        }

        // point size
        pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "obs_plot_pts");
    }
}

void MainWindow::plot_ctrl()
{
    double x_min = CONFIG::instance()->get_robot_size_x_min(); double x_max = CONFIG::instance()->get_robot_size_x_max();
    double y_min = CONFIG::instance()->get_robot_size_y_min(); double y_max = CONFIG::instance()->get_robot_size_y_max();
    double z_min = CONFIG::instance()->get_robot_size_z_min(); double z_max = CONFIG::instance()->get_robot_size_z_max();

    // plot global path
    if(is_global_path_update)
    {
        is_global_path_update = false;

        // draw
        PATH global_path = AUTOCONTROL::instance()->get_cur_global_path();
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < global_path.pos.size(); p++)
            {
                Eigen::Vector3d P = global_path.pos[p];

                pcl::PointXYZRGB pt;
                pt.x = P[0];
                pt.y = P[1];
                pt.z = P[2]+0.1;
                pt.r = 255;
                pt.g = 0;
                pt.b = 0;

                cloud->push_back(pt);
            }

            if(!pcl_viewer->updatePointCloud(cloud, "global_path"))
            {
                pcl_viewer->addPointCloud(cloud, "global_path");
            }
            pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, MAINWINDOW_INFO::plot_point_size*3, "global_path");
        }
    }

    // plot local path
    if(is_local_path_update)
    {
        is_local_path_update = false;

        // erase first
        for(size_t p = 0; p < last_plot_local_path.size(); p++)
        {
            QString name = last_plot_local_path[p];
            if(pcl_viewer->contains(name.toStdString()))
            {
                pcl_viewer->removeShape(name.toStdString());
            }
        }
        last_plot_local_path.clear();

        // draw local path
        PATH local_path = AUTOCONTROL::instance()->get_cur_local_path();

        // draw local path
        if(local_path.pos.size() >= 2)
        {
            for(size_t p = 0; p < local_path.pose.size(); p++)
            {
                if(p == local_path.pose.size()-1 || p % 50 == 0)
                {
                    QString name;
                    name.sprintf("local_path_%d", (int)p);

                    Eigen::Matrix4d tf = local_path.pose[p];
                    pcl_viewer->addCube(x_min, x_max,
                                        y_min, y_max,
                                        z_min, z_max, 0, 1, 0, name.toStdString());

                    pcl_viewer->updateShapePose(name.toStdString(), Eigen::Affine3f(tf.cast<float>()));
                    pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                            name.toStdString());
                    last_plot_local_path.push_back(name);
                }
            }
        }
    }

    // draw control points
    if(AUTOCONTROL::instance()->get_is_moving())
    {
        // erase first
        if(pcl_viewer->contains("cur_pos"))
        {
            pcl_viewer->removeShape("cur_pos");
        }

        if(pcl_viewer->contains("tgt_pos"))
        {
            pcl_viewer->removeShape("tgt_pos");
        }

        if(pcl_viewer->contains("local_goal"))
        {
            pcl_viewer->removeShape("local_goal");
        }

        // draw monitoring points
        {
            Eigen::Vector3d cur_pos    = AUTOCONTROL::instance()->get_last_cur_pos();
            Eigen::Vector3d tgt_pos    = AUTOCONTROL::instance()->get_last_tgt_pos();
            Eigen::Vector3d local_goal = AUTOCONTROL::instance()->get_last_local_goal();

            pcl_viewer->addSphere(pcl::PointXYZ(cur_pos[0], cur_pos[1], cur_pos[2]), 0.1, 1.0, 1.0, 1.0, "cur_pos");
            pcl_viewer->addSphere(pcl::PointXYZ(tgt_pos[0], tgt_pos[1], tgt_pos[2]), 0.1, 1.0, 0.0, 0.0, "tgt_pos");
            pcl_viewer->addSphere(pcl::PointXYZ(local_goal[0], local_goal[1], local_goal[2]), 0.1, 0.0, 1.0, 0.0, "local_goal");

            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cur_pos");
            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "tgt_pos");
            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "local_goal");
        }
    }
    else
    {
        if(pcl_viewer->contains("cur_pos"))
        {
            pcl_viewer->removeShape("cur_pos");
        }

        if(pcl_viewer->contains("tgt_pos"))
        {
            pcl_viewer->removeShape("tgt_pos");
        }

        if(pcl_viewer->contains("local_goal"))
        {
            pcl_viewer->removeShape("local_goal");
        }
    }

    // draw robot
    Eigen::Matrix4d cur_tf = LOCALIZATION::instance()->get_cur_tf();
    {
        // draw axis
        if(!pcl_viewer->contains("robot_axis"))
        {
            pcl_viewer->addCoordinateSystem(1.0, "robot_axis");
        }
        pcl_viewer->updateCoordinateSystemPose("robot_axis", Eigen::Affine3f(cur_tf.cast<float>()));

        // draw body
        if(!pcl_viewer->contains("robot_body"))
        {
            pcl_viewer->addCube(x_min, x_max,
                                y_min, y_max,
                                z_min, z_max, 0.0, 0.5, 1.0, "robot_body");
            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "robot_body");
        }
        pcl_viewer->updateShapePose("robot_body", Eigen::Affine3f(cur_tf.cast<float>()));

        // draw vel
        if(pcl_viewer->contains("vel_text"))
        {
            pcl_viewer->removeShape("vel_text");
        }

        Eigen::Vector3d control_input = MOBILE::instance()->get_control_input();
        QString text;
        text.sprintf("vx:%.2f, vy:%.2f, wz:%.2f, info:%s, zone:%s", control_input[0], control_input[1], control_input[2]*R2D
                , POLICY::instance()->get_cur_info().toLocal8Bit().data()
                , POLICY::instance()->get_cur_zone().toLocal8Bit().data());
        ui->lb_RobotVel->setText(text);

        // draw cur node
        if(pcl_viewer->contains("cur_node"))
        {
            pcl_viewer->removeShape("cur_node");
        }

        QString cur_node_id; // = ctrl.get_cur_node_id();
        if(cur_node_id != "")
        {
            NODE *node = UNIMAP::instance()->get_node_by_id(cur_node_id);
            if(node != nullptr)
            {
                Eigen::Matrix4d tf = node->tf;
                tf(2,3) += CONFIG::instance()->get_robot_size_z_max();
                pcl::PolygonMesh donut = make_donut(CONFIG::instance()->get_robot_radius()*0.5, 0.05, tf, 0.0, 1.0, 1.0);
                pcl_viewer->addPolygonMesh(donut, "cur_node");
            }
        }
    }


    // plot goal info
    ui->lb_RobotGoal->setText(QString("id:%1\ninfo:%2").arg(AUTOCONTROL::instance()->get_cur_move_info().goal_node_id).
                              arg(AUTOCONTROL::instance()->get_cur_move_state()));
}

void MainWindow::plot_cam()
{
    int cam_num = CONFIG::instance()->get_cam_num();
    for(int i = 0 ; i<cam_num;i++)
    {
        if(CAM::instance()->get_connection(i))
        {
            cv::Mat plot = CAM::instance()->get_time_img(i).img;
            if(!plot.empty())
            {
//                cv::flip(plot,plot,0);

                // QLabel name create dynamically
                QString labelName = QString("lb_Screen%1").arg(i+2); // lb_Screen2, lb_Screen3 ...
                QLabel* label = this->findChild<QLabel*>(labelName);

                if(label)
                {
                    label->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
                    label->setScaledContents(true);
                    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
                }
                else
                {
                    qDebug() << labelName << " not found!";
                }
            }
        }
    }

    /*
    if(CAM::instance()->get_connection(1))
    {
        cv::Mat plot = CAM::instance()->get_time_img(1).img;
        if(!plot.empty())
        {
            cv::flip(plot, plot, 0);

            ui->lb_Screen3->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
            ui->lb_Screen3->setScaledContents(true);
            ui->lb_Screen3->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        }
    }
    if(CAM::instance()->get_connection(2))
    {
        cv::Mat plot = CAM::instance()->get_time_img(2).img;
        if(!plot.empty())
        {
            cv::flip(plot, plot, 0);

            ui->lb_Screen4->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
            ui->lb_Screen4->setScaledContents(true);
            ui->lb_Screen4->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        }
    }
    if(CAM::instance()->get_connection(3))
    {
        cv::Mat plot = CAM::instance()->get_time_img(3).img;
        if(!plot.empty())
        {
            cv::flip(plot, plot, 0);

            ui->lb_Screen5->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
            ui->lb_Screen5->setScaledContents(true);
            ui->lb_Screen5->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        }
    }*/
}

void MainWindow::plot_tractile()
{
    double x_min = CONFIG::instance()->get_robot_size_x_min(); double x_max = CONFIG::instance()->get_robot_size_x_max();
    double y_min = CONFIG::instance()->get_robot_size_y_min(); double y_max = CONFIG::instance()->get_robot_size_y_max();
    double z_min = CONFIG::instance()->get_robot_size_z_min(); double z_max = CONFIG::instance()->get_robot_size_z_max();

    Eigen::Matrix4d cur_tf = LOCALIZATION::instance()->get_cur_tf();


    // plot tactile
    {
        // erase first
        if(last_plot_tactile.size() > 0)
        {
            for(size_t p = 0; p < last_plot_tactile.size(); p++)
            {
                QString name = last_plot_tactile[p];
                if(pcl_viewer->contains(name.toStdString()))
                {
                    pcl_viewer->removeShape(name.toStdString());
                }
            }
            last_plot_tactile.clear();
        }

        Eigen::Vector3d vel = MOBILE::instance()->get_pose().vel;
        std::vector<Eigen::Matrix4d> traj = AUTOCONTROL::instance()->calc_trajectory(vel, 0.2, 1.0, cur_tf);
        for(size_t p = 0; p < traj.size(); p++)
        {
            if(p == traj.size()-1 || p % 50 == 0)
            {
                QString name = QString("traj_%1").arg(p);
                std::string name_str = name.toStdString();

                pcl_viewer->addCube(x_min, x_max,
                                    y_min, y_max,
                                    z_min, z_max, 1.0, 0.0, 0.0, name_str);

                pcl_viewer->updateShapePose(name_str, Eigen::Affine3f(traj[p].cast<float>()));
                pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name_str);
                last_plot_tactile.push_back(name);
            }
        }
    }


    // plot obs tactile
    {
        // erase first
        if(last_obs_plot_tactile.size() > 0)
        {
            for(size_t p = 0; p < last_obs_plot_tactile.size(); p++)
            {
                QString name = last_obs_plot_tactile[p];
                if(pcl_viewer->contains(name.toStdString()))
                {
                    pcl_viewer->removeShape(name.toStdString());
                }
            }
            last_obs_plot_tactile.clear();
        }

        if(AUTOCONTROL::instance()->get_is_moving())
        {
            std::vector<Eigen::Matrix4d> traj = AUTOCONTROL::instance()->get_obs_traj();
            for(size_t p = 0; p < traj.size(); p++)
            {
                QString name = QString("traj_%1").arg(p);
                std::string name_str = name.toStdString();


                if(p == traj.size()-1 || p % 50 == 0)
                {
                    pcl_viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0.8, 0.8, 0.8, name_str);

                    pcl_viewer->updateShapePose(name_str, Eigen::Affine3f(traj[p].cast<float>()));
                    pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name_str);
                    last_obs_plot_tactile.push_back(name);
                }
            }
        }
    }
}

void MainWindow::plot_loop()
{
    plot_timer->stop();
    double st_time = get_time();

    if(!ui->ckb_PlotEnable->isChecked())
    {
        plot_process_time();
        plot_timer->start();
        return;
    }

    plot_map();
    plot_node();
    plot_pick();
    plot_info();
    plot_safety();
    plot_raw_2d();
    plot_raw_3d();
    plot_mapping();
    plot_loc();
    plot_obs();
    plot_ctrl();
    plot_cam();
    plot_tractile();
    plot_process_time();

    // camera reset
    if(is_set_top_view)
    {
        is_set_top_view = false;
        pcl_viewer->setCameraPosition(0, 0, 30, 0, 0, 0, 0, 1, 0);
        printf("[MAIN] set top view camera\n");
    }

    if(is_view_reset)
    {
        is_view_reset = false;
        pcl_viewer->resetCamera();
        printf("[MAIN] reset view camera\n");
    }

    // mapping view
    if(ui->cb_ViewType->currentText() == "VIEW_MAPPING")
    {
        Eigen::Matrix4d cur_tf = LOCALIZATION::instance()->get_cur_tf();
        Eigen::Matrix4d _cur_tf = se2_to_TF(TF_to_se2(cur_tf));
        _cur_tf(2,3) = cur_tf(2,3);

        Eigen::Matrix3d _cur_R = _cur_tf.block(0,0,3,3);
        Eigen::Vector3d _cur_t = _cur_tf.block(0,3,3,1);

        double d = ui->cb_ViewHeight->currentText().toDouble();
        Eigen::Vector3d cam_pos = _cur_R*Eigen::Vector3d(0,0,d) + _cur_t;
        Eigen::Vector3d cam_tgt = _cur_R*Eigen::Vector3d(0,0,0) + _cur_t;
        Eigen::Vector3d cam_up(1,0,0);

        pcl_viewer->setCameraPosition(cam_pos[0], cam_pos[1], cam_pos[2],
                cam_tgt[0], cam_tgt[1], cam_tgt[2],
                cam_up[0], cam_up[1], cam_up[2]);

        double near = 2.0;
        double far  = 2.0*d;
        pcl_viewer->setCameraClipDistances(near, far);
    }

    // rendering
    ui->qvtkWidget->renderWindow()->Render();

    // check plot drift
    plot_dt = get_time() - st_time;
    if(plot_dt > 0.1)
    {
        //printf("[MAIN] plot_loop, loop time drift: %f\n", (double)plot_proc_t);
    }

    plot_timer->start();
}

void MainWindow::speaker_handler(int speaker_cnt)
{
    MOBILE_STATUS ms = MOBILE::instance()->get_status();

    int speak_code = 0;

    // autodrive
    if(AUTOCONTROL::instance()->get_is_moving())
    {

        double obs_d = AUTOCONTROL::instance()->get_obs_dist();
        if(obs_d < 1.0)
        {
            speak_code = 10;
        }
        else if(obs_d < 2.0)
        {
            speak_code = 10;
        }
        else
        {
            speak_code = 2;
        }

        if(DOCKCONTROL::instance()->get_dock_fsm_state() != DOCKING_FSM_OFF)
        {
            speak_code = 3;
        }
    }
    else
    {
        if(ms.om_state == SM_OM_NORMAL_OP_AUTO || ms.om_state == SM_OM_NORMAL_OP_MANUAL)
        {
            speak_code = 11;
        }
    }


    if(speaker_cnt % 8 == 0)
    {
        MOBILE::instance()->sem_io_speaker(speak_code);
    }

    else if(speaker_cnt % 8 == 2)
    {
        MOBILE::instance()->sem_io_speaker(0);
    }
}

int MainWindow::led_handler()
{
    MOBILE_STATUS ms = MOBILE::instance()->get_status();

    int led_out = SAFETY_LED_OFF;
    if(ms.operational_stop_state_flag_1 || ms.operational_stop_state_flag_2)
    {
        led_out = SAFETY_LED_RED;
        return led_out;
    }

    // autodrive led control
    if(AUTOCONTROL::instance()->get_is_moving())
    {

        //charging
        if(ms.charge_state == CHARGING_STATION_CHARGING)
        {
            //docking process sucess
            led_out = SAFETY_LED_CONTRACTING_GREEN;
            return led_out;
        }

        //docking
        int dock_fsm_state_ = DOCKCONTROL::instance()->get_dock_fsm_state();
        if(!(dock_fsm_state_ == DOCKING_FSM_OFF))
        {
            //docking process..
            led_out = SAFETY_LED_WHITE_WAVERING;
            return led_out;
        }

        //autocontrol
        double obs_d = AUTOCONTROL::instance()->get_obs_dist();
        if(obs_d < 1.0)
        {
            led_out = SAFETY_LED_PURPLE_BLINKING;
            return led_out;

        }

        if(obs_d < 2.0)
        {
            led_out = SAFETY_LED_PURPLE;
            return led_out;

        }

        else
        {
            //normal auto drive led
            led_out = SAFETY_LED_GREEN_BLINKING;
            return led_out;
        }
    }

    // not autodrive led control
    else
    {
        if(ms.om_state == SM_OM_ROBOT_POWER_OFF)
        {
            // Pc boot up
            led_out = SAFETY_LED_YELLOW;

            // Brkae release
            if(ms.brake_release_sw == 1)
            {
                led_out = SAFETY_LED_BLUE;
                return led_out;
            }
        }
        else if((ms.om_state == SM_OM_NORMAL_OP_AUTO) || (ms.om_state == SM_OM_NORMAL_OP_MANUAL))
        {
            led_out = SAFETY_LED_CYAN;
            return led_out;
        }
    }

    return led_out;
}

void MainWindow::getIPv4()
{
    QString chosen = "N/A";

    for(const QHostAddress &addr : QNetworkInterface::allAddresses())
    {
        if(addr.protocol() == QAbstractSocket::IPv4Protocol && addr != QHostAddress::LocalHost)
        {
            ui->lb_RobotIP->setText(addr.toString());
            return;
        }
    }

    ui->lb_RobotIP->setText(chosen);
}
