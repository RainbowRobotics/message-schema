#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , config(this)
    , logger(this)
    , unimap(this)
    , obsmap(this)
    , mobile(this)
    , lidar_2d(this)
    , lidar_3d(this)
    , loc(this)
    , mapping(this)
    , ctrl(this)
    , plot_timer(this)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // for 3d viewer
    connect(ui->cb_ViewType, SIGNAL(currentIndexChanged(QString)), this, SLOT(all_update()));

    // for simulation
    connect(ui->bt_SimInit, SIGNAL(clicked()), this, SLOT(bt_SimInit()));

    // config
    connect(ui->bt_ConfigLoad, SIGNAL(clicked()), this, SLOT(bt_ConfigLoad()));

    // emergency stop
    connect(ui->bt_Emergency, SIGNAL(clicked()), this, SLOT(bt_Emergency()));

    // test
    connect(ui->bt_Sync, SIGNAL(clicked()), this, SLOT(bt_Sync()));
    connect(ui->bt_MoveLinearX, SIGNAL(clicked()), this, SLOT(bt_MoveLinearX()));
    connect(ui->bt_MoveLinearY, SIGNAL(clicked()), this, SLOT(bt_MoveLinearY()));
    connect(ui->bt_MoveRotate, SIGNAL(clicked()), this, SLOT(bt_MoveRotate()));

    // motor
    connect(ui->bt_MotorInit, SIGNAL(clicked()), this, SLOT(bt_MotorInit()));

    // jog
    connect(ui->bt_JogF, SIGNAL(pressed()), this, SLOT(bt_JogF()));
    connect(ui->bt_JogB, SIGNAL(pressed()), this, SLOT(bt_JogB()));
    connect(ui->bt_JogL, SIGNAL(pressed()), this, SLOT(bt_JogL()));
    connect(ui->bt_JogR, SIGNAL(pressed()), this, SLOT(bt_JogR()));
    connect(ui->bt_JogF, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogB, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogL, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogR, SIGNAL(released()), this, SLOT(bt_JogReleased()));

    connect(ui->bt_MapBuild, SIGNAL(clicked()), this, SLOT(bt_MapBuild()));
    connect(ui->bt_MapSave, SIGNAL(clicked()), this, SLOT(bt_MapSave()));
    connect(ui->bt_MapLoad, SIGNAL(clicked()), this, SLOT(bt_MapLoad()));
    connect(ui->bt_MapLastLc, SIGNAL(clicked()), this, SLOT(bt_MapLastLc()));

    // localization
    connect(ui->bt_LocInit, SIGNAL(clicked()), this, SLOT(bt_LocInit()));
    connect(ui->bt_LocStart, SIGNAL(clicked()), this, SLOT(bt_LocStart()));
    connect(ui->bt_LocStop, SIGNAL(clicked()), this, SLOT(bt_LocStop()));

    // for obsmap
    connect(&obsmap, SIGNAL(obs_updated()), this, SLOT(obs_update()), Qt::QueuedConnection);
    connect(ui->bt_ObsClear, SIGNAL(clicked()), this, SLOT(bt_ObsClear()));

    // for autocontrol
    connect(ui->bt_AutoMove, SIGNAL(clicked()), this, SLOT(bt_AutoMove()));
    connect(ui->bt_AutoMove2, SIGNAL(clicked()), this, SLOT(bt_AutoMove2()));
    connect(ui->bt_AutoMove3, SIGNAL(clicked()), this, SLOT(bt_AutoMove3()));
    connect(ui->bt_AutoStop, SIGNAL(clicked()), this, SLOT(bt_AutoStop()));
    connect(ui->bt_AutoPause, SIGNAL(clicked()), this, SLOT(bt_AutoPause()));
    connect(ui->bt_AutoResume, SIGNAL(clicked()), this, SLOT(bt_AutoResume()));
    connect(ui->bt_ReturnToCharging, SIGNAL(clicked()), this, SLOT(bt_ReturnToCharging()));
    connect(&ctrl, SIGNAL(signal_local_path_updated()), this, SLOT(slot_local_path_updated()), Qt::QueuedConnection);
    connect(&ctrl, SIGNAL(signal_global_path_updated()), this, SLOT(slot_global_path_updated()), Qt::QueuedConnection);

    // timer
    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));

    // set effect
    init_ui_effect();

    // set plot window
    setup_vtk();

    // init modules
    init_modules();

    // start plot loop
    plot_timer.start(50);
}

MainWindow::~MainWindow()
{
    plot_timer.stop();
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
    // clear picking info
    pick = PICKING();

    map_update();
    topo_update();
    pick_update();
    obs_update();
}

void MainWindow::init_ui_effect()
{
    // set opacity
    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_Screen1->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_Screen2->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_Screen3->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_Screen4->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_Screen5->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->bt_ViewLeft->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->bt_ViewRight->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->bt_ViewUp->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->bt_ViewDown->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->bt_ViewZoomIn->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->bt_ViewZoomOut->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_PickPose->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_NodeId->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_RobotVel->setGraphicsEffect(effect);
    }

    {
        QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(this);
        effect->setOpacity(0.75);
        ui->lb_RobotGoal->setGraphicsEffect(effect);
    }
}

void MainWindow::setup_vtk()
{
    // Set up the QVTK window
    {
        vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
        vtkNew<vtkRenderer> renderer;
        renderWindow->AddRenderer(renderer);
        viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
        ui->qvtkWidget->setRenderWindow(viewer->getRenderWindow());
        viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
        viewer->setBackgroundColor(1.0, 1.0, 1.0);
        viewer->resetCamera();

        ui->qvtkWidget->grabGesture(Qt::PinchGesture);
        ui->qvtkWidget->installEventFilter(this);

        // init drawing
        viewer->addCoordinateSystem(1.0, "O_global");
        ui->qvtkWidget->renderWindow()->Render();

        // install event filter
        ui->qvtkWidget->installEventFilter(this);
        ui->qvtkWidget->setMouseTracking(true);
    }
}

void MainWindow::init_modules()
{
    // load config
    if(config.load_common(QCoreApplication::applicationDirPath() + "/configs/common.json"))
    {
        config.config_path = QCoreApplication::applicationDirPath() + "/configs/" + config.PLATFORM_NAME + "/config.json";
        config.load();
    }
    else
    {
        QMessageBox::warning(this, "Config Load Failed", "Failed to load common config file.\nPlease check the path or configuration.");
    }

    // simulation check
    if(config.USE_SIM)
    {
        this->setWindowTitle("SLAMNAV2 (SIMULATION MODE)");

        QPalette palette = this->palette();
        palette.setColor(QPalette::Window, Qt::red);
        this->setPalette(palette);

        ui->bt_SimInit->setEnabled(true);
    }

    // view mode
    if(config.LOC_MODE == "3D")
    {
        ui->cb_ViewType->setCurrentText("VIEW_3D");
    }

    // log module init (slamnav)
    logger.log_path = QCoreApplication::applicationDirPath() + "/snlog/";
    logger.init();

    // unimap module init
    unimap.config = &config;
    unimap.logger = &logger;

    // obsmap module init
    obsmap.config = &config;
    obsmap.logger = &logger;
    obsmap.unimap = &unimap;
    obsmap.init();

    // mobile module init
    mobile.config = &config;
    mobile.logger = &logger;
    mobile.open();

    // lidar 2d module init
    if(config.USE_LIDAR_2D)
    {
        lidar_2d.config = &config;
        lidar_2d.logger = &logger;
        lidar_2d.mobile = &mobile;
        lidar_2d.init();
        lidar_2d.open();
    }

    // lidar 3d module init
    if(config.USE_LIDAR_3D)
    {
        lidar_3d.config = &config;
        lidar_3d.logger = &logger;
        lidar_3d.init();
        lidar_3d.open();
    }

    // localization module init
    loc.config = &config;
    loc.logger = &logger;
    loc.mobile = &mobile;
    loc.lidar_2d = &lidar_2d;
    loc.lidar_3d = &lidar_3d;
    // loc.cam = &cam;
    loc.unimap = &unimap;
    loc.obsmap = &obsmap;

    // mapping module init
    if(config.USE_LIDAR_2D)
    {
        mapping.config = &config;
        mapping.logger = &logger;
        mapping.unimap = &unimap;
        mapping.lidar_2d = &lidar_2d;
        mapping.loc = &loc;
    }

    // autocontrol module init
    ctrl.config = &config;
    ctrl.logger = &logger;
    ctrl.mobile = &mobile;
    // ctrl.lidar = &lidar;
    // ctrl.cam = &cam;
    ctrl.loc = &loc;
    ctrl.unimap = &unimap;
    ctrl.obsmap = &obsmap;
    ctrl.init();

    // simulation module init
    sim.config = &config;
    sim.logger = &logger;
    sim.unimap = &unimap;
    sim.mobile = &mobile;
    if(config.LOC_MODE == "3D")
    {
        sim.lidar_3d = &lidar_3d;
    }
    else
    {
        sim.lidar_2d = &lidar_2d;
    }
    sim.loc = &loc;

    // start jog loop
    jog_flag = true;
    jog_thread = new std::thread(&MainWindow::jog_loop, this);

    // start watchdog loop
    watch_flag = true;
    watch_thread = new std::thread(&MainWindow::watch_loop, this);

    // auto load
    if(config.MAP_PATH.isEmpty())
    {
        return;
    }
    unimap.load_map(config.MAP_PATH);
    all_update();
}

void MainWindow::all_plot_clear()
{
    viewer->removeAllCoordinateSystems();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // add global axis
    viewer->addCoordinateSystem(1.0, "O_global");
}

// for picking interface
bool MainWindow::eventFilter(QObject *object, QEvent *ev)
{
    if(object == ui->qvtkWidget && unimap.is_loaded == MAP_LOADED)
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

        // check plot enabled
        // if(ui->ckb_PlotEnable->isChecked() == false)
        // {
        //     return true;
        // }

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
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.cur_node = unimap.get_goal_id(pt);

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
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

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
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

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
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

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
                        picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

                        Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                        pick.cur_node = unimap.get_goal_id(pt);

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
                        picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

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
                        picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

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
                        picking_ray(x, y, w, h, ray_center, ray_direction, viewer);

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
    viewer->getCameras(cams);

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

    viewer->setCameraPosition(pos_new[0], pos_new[1], pos_new[2],
                              focal_new[0], focal_new[1], focal_new[2],
                              up_new[0], up_new[1], up_new[2]);


    viewer->setCameraClipDistances(2.0, 1000.0);
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
        logger.write_log("pcl viewer not have camera ..", "Red", true, false);
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
    last_jog_update_t = get_time();

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
    if(unimap.is_loaded != MAP_LOADED)
    {
        logger.write_log("[SIM] map load first", "Red", true, false);
        return;
    }

    // loc stop
    loc.stop();

    // stop first
    sim.stop();

    // set
    Eigen::Matrix4d tf = se2_to_TF(pick.r_pose);
    sim.set_cur_tf(tf);
    loc.set_cur_tf(tf);

    // start
    sim.start();

    // make dummy
    mobile.is_connected = true;
    mobile.is_synced = true;

    // lidar.is_connected_f = true;
    // lidar.is_connected_b = true;

    // lidar.is_synced_f = true;
    // lidar.is_synced_b = true;

    // loc start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    loc.start();
}

void MainWindow::bt_ConfigLoad()
{
    config.load();
}

// for emergency stop
void MainWindow::bt_Emergency()
{
    // task.cancel();
    ctrl.stop();
    ctrl.set_multi_req("none");
    ctrl.set_obs_condition("none");
    // dctrl.stop();
    mobile.move(0,0,0);
}

// mobile
void MainWindow::bt_MotorInit()
{
    mobile.motor_on();
}

void MainWindow::bt_Sync()
{
    mobile.sync();

    if(config.USE_LIDAR_2D)
    {
        lidar_2d.set_sync_flag(true);
    }

    if(config.USE_LIDAR_3D)
    {
        lidar_3d.set_sync_flag(true);
    }
}

void MainWindow::bt_MoveLinearX()
{
    ctrl.is_moving = true;
    QTimer::singleShot(1000, [&]()
    {
        double d = ui->dsb_MoveLinearDist->value();
        double v = ui->dsb_MoveLinearV->value();
        double t = std::abs(d/v) + 0.5;

        mobile.move_linear_x(d, v);

        QTimer::singleShot(t*1000, [&]()
        {
            ctrl.is_moving = false;
        });
    });
}

void MainWindow::bt_MoveLinearY()
{
    ctrl.is_moving = true;
    QTimer::singleShot(1000, [&]()
    {
        double d = ui->dsb_MoveLinearDist->value();
        double v = ui->dsb_MoveLinearV->value();
        double t = std::abs(d/v) + 0.5;

        mobile.move_linear_y(d, v);

        QTimer::singleShot(t*1000, [&]()
        {
            ctrl.is_moving = false;
        });
    });
}

void MainWindow::bt_MoveRotate()
{
    ctrl.is_moving = true;
    QTimer::singleShot(1000, [&]()
    {
        double th = ui->dsb_MoveRotateDeg->value() * D2R;
        double w = ui->dsb_MoveRotateW->value() * D2R;
        double t = std::abs(th/w) + 0.5;

        mobile.move_rotate(th, w);

        QTimer::singleShot(t*1000, [&]()
        {
            ctrl.is_moving = false;
        });
    });
}

void MainWindow::bt_MoveStop()
{
    ctrl.is_moving = false;
    mobile.stop();
}

void MainWindow::bt_JogF()
{
    is_jog_pressed = true;
    update_jog_values(ui->spb_JogV->value(), 0, 0);
}

void MainWindow::bt_JogB()
{
    is_jog_pressed = true;
    update_jog_values(-ui->spb_JogV->value(), 0, 0);
}

void MainWindow::bt_JogL()
{
    is_jog_pressed = true;
    update_jog_values(0, 0, ui->spb_JogW->value()*D2R);
}

void MainWindow::bt_JogR()
{
    is_jog_pressed = true;
    update_jog_values(0, 0, -ui->spb_JogW->value()*D2R);
}

void MainWindow::bt_JogReleased()
{
    is_jog_pressed = false;
    update_jog_values(0, 0, 0);
}

// mapping
void MainWindow::bt_MapBuild()
{
    if(config.USE_SIM == 1)
    {
        logger.write_log("[MAIN] map build not allowed SIM_MODE", "Red", true, false);
        return;
    }

    // stop first
    mapping.stop();

    // clear first
    unimap.clear();
    all_plot_clear();

    // auto generation dir path
    QString _map_dir = QDir::homePath() + "/maps/" + get_time_str();
    QDir().mkpath(_map_dir);
    unimap.map_dir = _map_dir;

    // mapping start
    mapping.start();
}

void MainWindow::bt_MapSave()
{
    if(config.USE_SIM == 1)
    {
        logger.write_log("[MAIN] map save not allowed SIM_MODE", "Red", true, false);
        return;
    }

    // check
    if(unimap.map_dir == "")
    {
        logger.write_log("[MAIN] no map_dir", "Red", true, false);
        return;
    }

    // stop first
    mapping.stop();

    // check kfrm
    if(mapping.kfrm_storage.size() == 0)
    {
        printf("[MAIN] no keyframe\n");
        return;
    }

    // get all points and sampling
    const int64_t p1 = 73856093;
    const int64_t p2 = 19349669;
    const int64_t p3 = 83492791;
    const double voxel_size = config.SLAM_VOXEL_SIZE;

    std::unordered_map<int64_t, uint8_t> hash_map;
    std::vector<PT_XYZR> pts;
    for(size_t p = 0; p < mapping.kfrm_storage.size(); p++)
    {
        Eigen::Matrix4d G = mapping.kfrm_storage[p].opt_G;
        for(size_t q = 0; q < mapping.kfrm_storage[p].pts.size(); q++)
        {
            Eigen::Vector3d P;
            P[0] = mapping.kfrm_storage[p].pts[q].x;
            P[1] = mapping.kfrm_storage[p].pts[q].y;
            P[2] = mapping.kfrm_storage[p].pts[q].z;

            Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);

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
                pt.r = mapping.kfrm_storage[p].pts[q].r;
                pts.push_back(pt);
            }
        }

        printf("[MAIN] convert: %d/%d ..\n", (int)(p+1), (int)mapping.kfrm_storage.size());
    }

    // write file
    QString cloud_csv_path = unimap.map_dir + "/cloud.csv";
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
        printf("[MAIN] %s saved\n", cloud_csv_path.toLocal8Bit().data());
    }
}

void MainWindow::bt_MapLoad()
{
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    if(!path.isNull())
    {
        // clear first
        loc.stop();
        obsmap.clear();

        config.set_map_path(path);

        unimap.load_map(path);
        all_update();
    }
}

void MainWindow::bt_MapLastLc()
{
    mapping.last_lc();
}

// localization
void MainWindow::bt_LocInit()
{
    loc.set_cur_tf(se2_to_TF(pick.r_pose));
}

void MainWindow::bt_LocStart()
{
    loc.start();
}

void MainWindow::bt_LocStop()
{
    loc.stop();
}

// for autocontrol
void MainWindow::bt_AutoMove()
{
    if(unimap.is_loaded != MAP_LOADED)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    if(pick.cur_node != "")
    {
        NODE* node = unimap.get_node_by_id(pick.cur_node);
        if(node != NULL)
        {
            // just test ...
            PATH global_path = ctrl.calc_global_path(node->tf);
            if(global_path.pos.size() < 2)
            {
                printf("[ETA] eta:0\n");
            }
            else
            {
                // time align
                CTRL_PARAM params = ctrl.load_preset(0);
                Eigen::Matrix4d cur_tf = loc.get_cur_tf();
                auto dtdr_st = dTdR(cur_tf, global_path.pose.front());
                double time_align = (dtdr_st[1] / (params.LIMIT_W*D2R + 1e-06)) * 2; // first align + final align

                // time driving
                double time_driving = 0.0;
                for(size_t p = 1; p < global_path.pos.size()-1; p++)
                {
                    Eigen::Vector3d pos0 = global_path.pos[p];
                    Eigen::Vector3d pos1 = global_path.pos[p+1];
                    double dist = (pos1 - pos0).norm();

                    double ref_v0 = global_path.ref_v[p];
                    double ref_v1 = global_path.ref_v[p+1];
                    double v = (ref_v0 + ref_v1)/2;

                    time_driving += dist / (v+1e-06);
                }

                if(time_driving == 0.0)
                {
                    time_driving = 9999.0;
                }

                printf("[ETA] eta-> driving:%f, align:%f, total:%f\n", time_driving, time_align, (time_driving + time_align));
            }

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

            Q_EMIT ctrl.signal_move(msg);
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

        Q_EMIT ctrl.signal_move(msg);

        return;
    }
}

void MainWindow::bt_AutoMove2()
{
    if(unimap.is_loaded == false)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    if(pick.cur_node != "")
    {
        NODE* node = unimap.get_node_by_id(pick.cur_node);
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
            ctrl.move(msg);
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
        ctrl.move(msg);

        return;
    }
}

void MainWindow::bt_AutoMove3()
{

}

void MainWindow::bt_AutoStop()
{
    // stop driving
    ctrl.stop();
}

void MainWindow::bt_AutoPause()
{
    ctrl.is_pause = true;
}

void MainWindow::bt_AutoResume()
{
    ctrl.is_pause = false;
}

void MainWindow::bt_ReturnToCharging()
{
    if(config.ROBOT_SERIAL_NUMBER == "RB-M-")
    {
        logger.write_log("[RTC] Robot serial number is not properly set.", "Red");
        return;
    }

    std::vector<QString> found_ids;

    for(size_t i = 0; i < unimap.nodes.size(); i++)
    {
        if(unimap.nodes[i].name.contains(config.ROBOT_SERIAL_NUMBER))
        {
            found_ids.push_back(unimap.nodes[i].id);
        }
    }

    if(found_ids.size() == 0)
    {
        logger.write_log(QString("[RTC] No charging node found for robot serial: %1").arg(config.ROBOT_SERIAL_NUMBER), "Red");
        return;
    }

    if(found_ids.size() > 1)
    {
        logger.write_log(QString("[RTC] Multiple charging nodes found for serial: %1 (count: %2)")
                         .arg(config.ROBOT_SERIAL_NUMBER)
                         .arg(found_ids.size()), "Red");
        return;
    }


    QString found_id = found_ids[0];
    NODE* node = unimap.get_node_by_id(found_id);
    if(node == NULL)
    {
        logger.write_log(QString("[RTC] Failed to retrieve node with ID: %1").arg(found_id), "Red");
        return;
    }

    logger.write_log(QString("[RTC] Found charging node: %1").arg(node->name), "Green");

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

    Q_EMIT ctrl.signal_move(msg);
}

void MainWindow::slot_local_path_updated()
{
    is_local_path_update = true;
}

void MainWindow::slot_global_path_updated()
{
    is_global_path_update = true;
}

// for obsmap
void MainWindow::bt_ObsClear()
{
    obsmap.clear();
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
        if(ctrl.is_moving == false || ctrl.is_debug == true)
        {
            // action
            double v_acc = ui->spb_AccV->value();
            double v_decel = ui->spb_DecelV->value();

            double w_acc = ui->spb_AccW->value()*D2R;
            double w_decel = ui->spb_DecelW->value()*D2R;

            double vx = apply_jog_acc(mobile.vx0, vx_target, v_acc, v_decel, dt);
            double vy = apply_jog_acc(mobile.vy0, vy_target, v_acc, v_decel, dt);
            double wz = apply_jog_acc(mobile.wz0, wz_target, w_acc, w_decel, dt);
            if(is_jog_pressed == false && get_time() - last_jog_update_t > 1.0)
            {
                if(vx_target != 0 || vy_target != 0 || wz_target != 0)
                {
                    vx_target = 0;
                    vy_target = 0;
                    wz_target = 0;
                    printf("[JOG] no input, stop\n");
                }
            }
            mobile.move(vx, vy, wz);
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
            //printf("[JOG] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    printf("[JOG] loop stop\n");
}

// watchdog
void MainWindow::watch_loop()
{
    int prevent_duplicate_clicks_cnt =0;
    int cnt = 0;
    int loc_fail_cnt = 0;
    double last_sync_time = 0;
    bool last_fms_connection = false;

    CPU_USAGE pre_cpu_usage;

    bool is_first_emo_check = true;
    MOBILE_STATUS pre_ms = mobile.get_status();

    printf("[WATCHDOG] loop start\n");
    while(watch_flag)
    {
        cnt++;

        // every 10 sec
        // if(cnt % 100 == 0)
        // {
        //     if(!QCoreApplication::instance()->thread()->isRunning())
        //     {
        //         logger.write_log("[MAIN] Main thread is unresponsive!", "Red");
        //     }
        //     else
        //     {
        //         if(config.USE_COMM_FMS)
        //         {
        //             printf("[MAIN] robot id: %s\n", comm_fms.robot_id.toLocal8Bit().data());
        //         }
        //     }
        // }

        // for 500ms
        if(cnt % 2 == 0)
        {
            if(mobile.is_connected)
            {
                MOBILE_STATUS ms = mobile.get_status();
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
                            Eigen::Vector3d cur_pos = loc.get_cur_tf().block(0,3,3,1);

                            DATA_MOVE move_info;
                            move_info.cur_pos = cur_pos;
                            move_info.result = "emo";
                            move_info.message = "released";
                            move_info.time = get_time();

                            Q_EMIT signal_move_response(move_info);
                        }
                        else if(pre_ms.motor_stop_state >= 1 && ms.motor_stop_state == 0)
                        {
                            Eigen::Vector3d cur_pos = loc.get_cur_tf().block(0,3,3,1);

                            DATA_MOVE move_info;
                            move_info.cur_pos = cur_pos;
                            move_info.result = "emo";
                            move_info.message = "pushed";
                            move_info.time = get_time();

                            Q_EMIT signal_move_response(move_info);
                        }

                        pre_ms = ms;
                    }
                }
            }
        }

        // for 1000ms loop
        if(cnt % 10 == 0)
        {
            // led state
            int led_color = LED_GREEN;

            // autodrive led control
            if(ctrl.is_moving)
            {
                if(ctrl.get_obs_condition() == "far")
                {
                    led_color = LED_WHITE_BLINK;
                }
                else if(ctrl.get_obs_condition() == "near")
                {
                    led_color = LED_RED;

                    if(config.USE_BEEP)
                    {
                        QApplication::beep();
                    }
                }
                else if(ctrl.get_obs_condition() == "near_robot")
                {
                    led_color = LED_RED;
                }
                else
                {
                    led_color = LED_WHITE;
                }
            }

            // check mobile
            if(mobile.is_connected)
            {
                if(mobile.is_synced == false)
                {
                    mobile.sync();
                    last_sync_time = get_time();
                    logger.write_log("[WATCH] try time sync, pc and mobile");
                }

                MOBILE_STATUS ms = mobile.get_status();
                if(ms.t != 0)
                {
                    // when motor status 0, emo released, no charging
                    if((ms.status_m0 == 0 || ms.status_m1 == 0) && ms.motor_stop_state == 1 && ms.charge_state == 0)
                    {
                        mobile.motor_on();
                    }

                    if(ms.connection_m0 == 1 && ms.connection_m1 == 1 &&
                       ms.status_m0 == 1 && ms.status_m1 == 1 &&
                       ms.motor_stop_state == 1 && ms.charge_state == 0)
                    {
                        mobile.set_cur_pdu_state("good");
                    }
                    else
                    {
                        mobile.set_cur_pdu_state("fail");
                        led_color = LED_MAGENTA;
                    }
                }
            }

            // check lidar 3d
            if(config.USE_LIDAR_3D)
            {
                if(lidar_3d.is_connected && !lidar_3d.is_sync)
                {
                    lidar_3d.set_sync_flag(true);
                    QString str = QString("[WATCH] try time sync, pc and lidar, type: %1").arg(config.LIDAR_3D_TYPE);
                    printf("%s\n", str.toLocal8Bit().data());
                }
            }

            // check lidar 2d
            if(config.USE_LIDAR_2D)
            {
                if(lidar_2d.is_connected && !lidar_2d.is_sync)
                {
                    lidar_2d.set_sync_flag(true);
                    QString str = QString("[WATCH] try time sync, pc and lidar, type: %1").arg(config.LIDAR_2D_TYPE);
                    printf("%s\n", str.toLocal8Bit().data());
                }
            }

            // resync for time skew
            if(get_time() - last_sync_time > 1200.0 && ctrl.is_moving == false)
            {
                printf("[WATCH] resync all\n");

                if(mobile.is_connected)
                {
                    mobile.sync();
                }

                if(lidar_2d.is_connected)
                {
                    lidar_2d.set_sync_flag(true);
                }

                if(lidar_3d.is_connected)
                {
                    lidar_3d.set_sync_flag(true);
                }

                last_sync_time = get_time();
            }

            // check loc
            if(config.USE_SIM)
            {
                loc.set_cur_loc_state("good");
            }
            else
            {
                if(loc.is_loc == false)
                {
                    loc_fail_cnt = 0;
                    loc.set_cur_loc_state("none");
                    led_color = LED_MAGENTA;
                }
                else
                {
                    Eigen::Vector2d ieir = loc.get_cur_ieir();
                    if(ieir[0] > config.LOC_CHECK_IE || ieir[1] < config.LOC_CHECK_IR)
                    {
                        loc_fail_cnt++;
                        if(loc_fail_cnt > 3)
                        {
                            loc.set_cur_loc_state("fail");
                            led_color = LED_MAGENTA_BLINK;
                        }
                    }
                    else
                    {
                        loc_fail_cnt = 0;
                        loc.set_cur_loc_state("good");
                    }
                }
            }

            // check system info
            /*
            {
                MOBILE_STATUS ms = mobile.get_status();

                QString temp_str;
                QString power_str;
                QString cpu_usage_str;

                // check temperature
                {
                    int cpu_temp_sum = 0;
                    int _cnt = 0;

                    QDir dir_temp("/sys/class/thermal");
                    QStringList filters;
                    filters << "thermal_zone*";
                    dir_temp.setNameFilters(filters);
                    QFileInfoList list_temp = dir_temp.entryInfoList();

                    for (const QFileInfo &info : list_temp)
                    {
                        QString path = info.filePath() + "/temp";
                        QFile file(path);
                        if(file.open(QIODevice::ReadOnly | QIODevice::Text))
                        {
                            QTextStream in(&file);
                            QString line = in.readLine();
                            file.close();

                            bool ok = false;
                            int temperature = line.toInt(&ok);
                            if(ok)
                            {
                                if(temperature != 0)
                                {
                                    cpu_temp_sum += temperature/1000;
                                    _cnt++;
                                }
                            }
                        }
                    }

                    if(_cnt != 0)
                    {
                        int pc_temp = cpu_temp_sum/_cnt;
                        temp_str.sprintf("[TEMP] cpu:%d, m0:%d, m1:%d", pc_temp, ms.temp_m0, ms.temp_m1);
                        if(pc_temp > 85)
                        {
                            led_color = LED_RED_BLINK;
                        }
                    }
                    else
                    {
                        temp_str.sprintf("[TEMP] cpu:0, m0:%d, m1:d", ms.temp_m0, ms.temp_m1);
                    }
                }

                // check power
                {
                    int cpu_power_sum = 0;
                    int _cnt = 0;

                    QDir dir_temp("/sys/class/power_supply");
                    QStringList filters;
                    filters << "BAT*";
                    dir_temp.setNameFilters(filters);
                    QFileInfoList list_temp = dir_temp.entryInfoList();

                    for (const QFileInfo &info : list_temp)
                    {
                        QString path = info.filePath() + "/power_now";
                        QFile file(path);

                        if(file.open(QIODevice::ReadOnly | QIODevice::Text))
                        {
                            QTextStream in(&file);
                            QString line = in.readLine();
                            file.close();

                            bool ok = false;
                            int power = line.toInt(&ok);
                            if(ok)
                            {
                                if(power != 0)
                                {
                                    cpu_power_sum += power/1000;
                                    _cnt++;
                                }
                            }
                        }
                    }

                    if(_cnt != 0)
                    {
                        power_str.sprintf("[POWER] cpu:%.2f, m0:%.2f, m1:%.2f", (double)cpu_power_sum/_cnt, (double)ms.cur_m0/10.0, (double)ms.cur_m1/10.0);
                    }
                    else
                    {
                        power_str.sprintf("[POWER] cpu:0.0, m0:%.2f, m1:%.2f", (double)ms.cur_m0/10.0, (double)ms.cur_m1/10.0);
                    }
                }

                // check cpu usage
                {
                    QFile file("/proc/stat");
                    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
                    {
                        QTextStream in(&file);
                        QString line = in.readLine();
                        file.close();

                        CPU_USAGE cur_cpu_usage;
                        QStringList values = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);
                        if (values[0] == "cpu")
                        {
                            cur_cpu_usage.user = values[1].toLongLong();
                            cur_cpu_usage.nice = values[2].toLongLong();
                            cur_cpu_usage.system = values[3].toLongLong();
                            cur_cpu_usage.idle = values[4].toLongLong();
                            cur_cpu_usage.iowait = values[5].toLongLong();
                            cur_cpu_usage.irq = values[6].toLongLong();
                            cur_cpu_usage.softirq = values[7].toLongLong();
                        }

                        double cur_total_usage = cur_cpu_usage.user + cur_cpu_usage.nice + cur_cpu_usage.system + cur_cpu_usage.idle + cur_cpu_usage.iowait + cur_cpu_usage.irq + cur_cpu_usage.softirq;
                        double pre_total_usage = pre_cpu_usage.user + pre_cpu_usage.nice + pre_cpu_usage.system + pre_cpu_usage.idle + pre_cpu_usage.iowait + pre_cpu_usage.irq + pre_cpu_usage.softirq;

                        double cur_idle_usage = cur_cpu_usage.idle;
                        double pre_idle_usage = pre_cpu_usage.idle;

                        double cpu_usage = (1.0 - (double)(cur_idle_usage - pre_idle_usage) / (cur_total_usage - pre_total_usage)) * 100.0;
                        if(isfinite(cpu_usage))
                        {
                            cpu_usage_str.sprintf("[CPU]: %.2f", cpu_usage);
                            pre_cpu_usage = cur_cpu_usage;
                        }
                    }
                    else
                    {
                        cpu_usage_str.sprintf("[CPU]: 0.0");
                    }
                }

                // check emo
                {
                    if(ms.motor_stop_state == 0)
                    {
                        led_color = LED_YELLOW;
                    }
                }

                // check battery
                if(config.PLATFORM_NAME == "S100")
                {
                    if(ms.bat_out < (double)S100_BAT_MIN_VOLTAGE) // low battery
                    {
                        led_color = LED_YELLOW_BLINK;
                        // logger.write_log("[BATTERY] need charge");
                    }
                    else if(ms.charge_state == 1) // charge
                    {
                        led_color = LED_RED;
                        if(ms.bat_out > (double)S100_BAT_MAX_VOLTAGE)
                        {
                            led_color = LED_BLUE;
                        }
                    }

                    QString system_info_str = "[SYSTEM_INFO]\n" + temp_str + "\n" + power_str + "\n" + cpu_usage_str;
                    ui->lb_SystemInfo->setText(system_info_str);
                }
            }
            */

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[WATCHDOG] loop stop\n");
}


// for plot loop
void MainWindow::plot_map()
{
    if(unimap.is_loaded != MAP_LOADED)
    {
        return;
    }

    if(is_map_update)
    {
        is_map_update = false;

        // 2D
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < unimap.kdtree_cloud.pts.size(); p+=4) // sampling
            {
                if(unimap.kdtree_mask[p] == 0)
                {
                    continue;
                }

                pcl::PointXYZRGB pt;
                pt.x = unimap.kdtree_cloud.pts[p].x;
                pt.y = unimap.kdtree_cloud.pts[p].y;
                pt.z = unimap.kdtree_cloud.pts[p].z;

                double reflect = std::sqrt(unimap.kdtree_cloud.pts[p].r/255);
                tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                pt.r = c.r()*255;
                pt.g = c.g()*255;
                pt.b = c.b()*255;

                cloud->push_back(pt);
            }

            if(!viewer->updatePointCloud(cloud, "map_pts"))
            {
                viewer->addPointCloud(cloud, "map_pts");
            }

            // point size
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, "map_pts");
        }

        // 3D
        if(ui->cb_ViewType->currentText() == "VIEW_3D")
        {
            unimap.mtx.lock();
            std::vector<Eigen::Vector3d> pts = unimap.map_pts;
            std::vector<Eigen::Vector3d> nor = unimap.map_nor;
            std::vector<double> reflects = unimap.map_reflects;
            unimap.mtx.unlock();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < pts.size(); p++)
            {
                Eigen::Vector3d P = pts[p];
                double reflect = std::sqrt(reflects[p]/255);
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
            if(!viewer->updatePointCloud(cloud, "map_3d_pts"))
            {
                viewer->addPointCloud(cloud, "map_3d_pts");
            }

            // point size
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_3d_pts");
        }
        else
        {
            if(viewer->contains("map_3d_pts"))
            {
                viewer->removePointCloud("map_3d_pts");
            }
        }
    }
}

void MainWindow::plot_topo()
{
    if(unimap.is_loaded != MAP_LOADED)
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
                if(viewer->contains(id.toStdString()))
                {
                    viewer->removeShape(id.toStdString());
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
                if(viewer->contains(id.toStdString()))
                {
                    viewer->removeShape(id.toStdString());
                }
            }
            last_plot_names.clear();
        }

        // draw
        if(unimap.nodes.size() > 0)
        {
            if(ui->ckb_PlotNodes->isChecked())
            {
                // draw nodes
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(size_t p = 0; p < unimap.nodes.size(); p++)
                {
                    QString id = unimap.nodes[p].id;
                    if(viewer->contains(id.toStdString()))
                    {
                        printf("duplicate: %s\n", id.toStdString().data());
                    }

                    if(unimap.nodes[p].type == "ROUTE")
                    {
                        const double size = 0.1;
                        viewer->addCube(-size, size, -size, size, 0, size, 0.8, 0.8, 0.8, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        Eigen::Vector3d front_dot = unimap.nodes[p].tf.block(0,0,3,3)*Eigen::Vector3d(size - 0.05, 0, 0) + unimap.nodes[p].tf.block(0,3,3,1);

                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + size;
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(unimap.nodes[p].type == "INIT")
                    {
                        viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                        0.0, 0.1, 0.0, 1.0, 1.0, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        Eigen::Vector3d front_dot = unimap.nodes[p].tf.block(0,0,3,3)*Eigen::Vector3d(config.ROBOT_SIZE_X[1] - 0.05, 0, 0) + unimap.nodes[p].tf.block(0,3,3,1);

                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + config.ROBOT_SIZE_Z[1];
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(unimap.nodes[p].type == "GOAL")
                    {
                        viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                        0, 0.1, 0.5, 1.0, 0.0, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        Eigen::Vector3d front_dot = unimap.nodes[p].tf.block(0,0,3,3)*Eigen::Vector3d(config.ROBOT_SIZE_X[1] - 0.05, 0, 0) + unimap.nodes[p].tf.block(0,3,3,1);

                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + config.ROBOT_SIZE_Z[1];
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(unimap.nodes[p].type == "STATION")
                    {
                        viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                        0, 0.1, 0.5, 1.0, 0.5, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        Eigen::Vector3d front_dot = unimap.nodes[p].tf.block(0,0,3,3)*Eigen::Vector3d(config.ROBOT_SIZE_X[1] - 0.05, 0, 0) + unimap.nodes[p].tf.block(0,3,3,1);

                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + config.ROBOT_SIZE_Z[1];
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }
                    else if(unimap.nodes[p].type == "OBS")
                    {
                        QString info = unimap.nodes[p].info;

                        NODE_INFO res;
                        if(parse_info(info, "SIZE", res))
                        {
                            viewer->addCube(-res.sz[0]/2, res.sz[0]/2,
                                            -res.sz[1]/2, res.sz[1]/2,
                                            -res.sz[2]/2, res.sz[2]/2, 1.0, 0.0, 1.0, id.toStdString());

                            viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, id.toStdString());
                        }
                    }
                    else if(unimap.nodes[p].type == "ZONE")
                    {
                        QString info = unimap.nodes[p].info;

                        NODE_INFO res;
                        if(parse_info(info, "SIZE", res))
                        {
                            viewer->addCube(-res.sz[0]/2, res.sz[0]/2,
                                            -res.sz[1]/2, res.sz[1]/2,
                                            -res.sz[2]/2, res.sz[2]/2, 0.5, 1.0, 0.0, id.toStdString());

                            viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, id.toStdString());
                        }
                    }
                    else if(unimap.nodes[p].type == "ARUCO")
                    {
                        const double size = 0.15;
                        viewer->addCube(-size, size, -size, size, -size, size, 0.0, 1.0, 1.0, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        Eigen::Vector3d front_dot = unimap.nodes[p].tf.block(0,0,3,3)*Eigen::Vector3d(size - 0.05, 0, 0) + unimap.nodes[p].tf.block(0,3,3,1);

                        pcl::PointXYZRGB pt;
                        pt.x = front_dot[0];
                        pt.y = front_dot[1];
                        pt.z = front_dot[2] + size;
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;

                        cloud->push_back(pt);
                    }

                    // #if defined(USE_MECANUM)
                    // if(unimap.nodes[p].type == "GOAL" || unimap.nodes[p].type == "INIT" || unimap.nodes[p].type == "STATION")
                    // {
                    //     // plot text
                    //     QString name = unimap.nodes[p].name;
                    //     QString text_id = id + "_text";
                    //     pcl::PointXYZ position;
                    //     position.x = unimap.nodes[p].tf(0,3);
                    //     position.y = unimap.nodes[p].tf(1,3);
                    //     position.z = unimap.nodes[p].tf(2,3) + 1.5;
                    //     viewer->addText3D(name.toStdString(), position, 0.2, 0.0, 0.0, 0.0, text_id.toStdString());

                    //     last_plot_names.push_back(text_id);
                    // }
                    // #endif

                    // for erase
                    last_plot_nodes.push_back(id);
                }

                if(!viewer->updatePointCloud(cloud, "front_dots"))
                {
                    viewer->addPointCloud(cloud, "front_dots");
                }
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE*2, "front_dots");
            }

            if(ui->ckb_PlotEdges->isChecked())
            {
                // draw edges
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(size_t p = 0; p < unimap.nodes.size(); p++)
                {
                    for(size_t q = 0; q < unimap.nodes[p].linked.size(); q++)
                    {
                        NODE* node = unimap.get_node_by_id(unimap.nodes[p].linked[q]);
                        if(node == NULL)
                        {
                            continue;
                        }

                        Eigen::Vector3d P0 = unimap.nodes[p].tf.block(0,3,3,1);
                        Eigen::Vector3d P1 = node->tf.block(0,3,3,1);
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

                if(!viewer->updatePointCloud(cloud, "edge_dots"))
                {
                    viewer->addPointCloud(cloud, "edge_dots");
                }
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE*2, "edge_dots");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "edge_dots");

                if(!viewer->updatePointCloud(cloud2, "edge_chk_dots"))
                {
                    viewer->addPointCloud(cloud2, "edge_chk_dots");
                }
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE*2.5, "edge_chk_dots");
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
        if(viewer->contains("O_pick"))
        {
            viewer->removeCoordinateSystem("O_pick");
        }

        if(viewer->contains("pick_body"))
        {
            viewer->removeShape("pick_body");
        }

        if(viewer->contains("sel_cur"))
        {
            viewer->removeShape("sel_cur");
        }

        if(pick.cur_node != "")
        {
            NODE *node = unimap.get_node_by_id(pick.cur_node);
            if(node != NULL)
            {
                pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 1.0, 1.0, 1.0);
                viewer->addPolygonMesh(donut, "sel_cur");

                // plot text
                QString text = "id: " + node->id + ", name: " + node->name;
                ui->lb_NodeId->setText(text);
            }
        }

        if(pick.last_btn == 1)
        {
            Eigen::Matrix4d pick_tf = se2_to_TF(pick.r_pose);

            // draw pose
            viewer->addCoordinateSystem(1.0, "O_pick");
            viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                            config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                            config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.5, 0.5, 0.5, "pick_body");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");

            viewer->updateShapePose("pick_body", Eigen::Affine3f(pick_tf.cast<float>()));
            viewer->updateCoordinateSystemPose("O_pick", Eigen::Affine3f(pick_tf.cast<float>()));

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
        ui->lb_MobileStatusInfo->setText(mobile.get_status_text());

        if(mobile.is_connected)
        {
            // plot mobile pose
            ui->lb_MobilePoseInfo->setText(mobile.get_pose_text());

            // plot mobile status
            ui->lb_MobileStatusInfo->setText(mobile.get_status_text());
        }
    }

    // plot slam info
    {
        // mapping
        QString text;
        if(mapping.is_mapping)
        {
            text = mapping.get_info_text();
        }

        // localization
        if(loc.is_loc)
        {
            text += "\n" + loc.get_info_text();
        }

        ui->lb_SlamInfo->setText(text);
    }

    // 3d lidar info
    if(config.USE_LIDAR_2D)
    {
        ui->lb_LidarInfo_2D->setText(lidar_2d.get_info_text());
    }

    // 3d lidar info
    if(config.USE_LIDAR_3D)
    {
        ui->lb_LidarInfo_3D->setText(lidar_3d.get_info_text());
    }

    // plot auto info
    {
        QString _multi_state = "";
        int fsm_state = ctrl.fsm_state;

        QString auto_info_str;
        auto_info_str.sprintf("[AUTO_INFO]\nfsm_state: %s\nis_moving: %s, is_pause: %s, obs: %s\nis_multi: %d, request: %s, multi_state: %s",
                              AUTO_FSM_STATE_STR[fsm_state].toLocal8Bit().data(),
                              (bool)ctrl.is_moving ? "1" : "0",
                              (bool)ctrl.is_pause ? "1" : "0",
                              ctrl.get_obs_condition().toLocal8Bit().data(),
                              config.USE_MULTI,
                              ctrl.get_multi_req().toLocal8Bit().data(),
                              _multi_state.toLocal8Bit().data());
        ui->lb_AutoInfo->setText(auto_info_str);
    }
}

void MainWindow::plot_raw_2d()
{
    if(lidar_2d.is_connected && ui->cb_ViewType->currentText() == "VIEW_2D" && !mapping.is_mapping && !loc.is_loc)
    {
        for(int idx = 0; idx < config.LIDAR_2D_NUM; idx++)
        {
            RAW_FRAME cur_frm = lidar_2d.get_cur_raw(idx);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < cur_frm.pts.size(); p++)
            {
                Eigen::Vector3d P;
                P[0] = cur_frm.pts[p][0];
                P[1] = cur_frm.pts[p][1];
                P[2] = cur_frm.pts[p][2];

                Eigen::Vector3d _P = loc.get_cur_tf().block(0,0,3,3)*P + loc.get_cur_tf().block(0,3,3,1);

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

            QString cloud_id = QString("lidar_2d_cur_pts_%1").arg(idx);
            if(!viewer->updatePointCloud(cloud, cloud_id.toStdString()))
            {
                viewer->addPointCloud(cloud, cloud_id.toStdString());
            }
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id.toStdString());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, cloud_id.toStdString());
        }
    }
    else
    {
        // remove 3d lidar cloud
        for(int idx = 0; idx < config.LIDAR_2D_NUM; idx++)
        {
            QString cloud_id = QString("lidar_2d_cur_pts_%1").arg(idx);
            std::string id = cloud_id.toStdString();
            if(viewer->contains(id))
            {
                viewer->removePointCloud(id);
            }
        }
    }
}

void MainWindow::plot_raw_3d()
{
    // plot 3d lidar
    if(lidar_3d.is_connected && ui->cb_ViewType->currentText() == "VIEW_3D" && !mapping.is_mapping && !loc.is_loc)
    {
        for(int idx = 0; idx < config.LIDAR_3D_NUM; idx++)
        {
            // plot current lidar data
            LVX_FRM cur_frm = lidar_3d.get_cur_raw(idx);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < cur_frm.pts.size(); p++)
            {
                Eigen::Vector3d P;
                P[0] = cur_frm.pts[p].x;
                P[1] = cur_frm.pts[p].y;
                P[2] = cur_frm.pts[p].z;

                Eigen::Vector3d _P = loc.get_cur_tf().block(0,0,3,3)*P + loc.get_cur_tf().block(0,3,3,1);


                pcl::PointXYZRGB pt;
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                // pt.r = lidar_color.r;
                // pt.g = lidar_color.g;
                // pt.b = lidar_color.b;
                if(idx == 0)
                {
                    pt.r = 255; pt.g = 127; pt.b = 0;   // Orange for lidar 0
                }
                else
                {
                    pt.r = 0; pt.g = 200; pt.b = 255;   // Cyan for lidar 1
                }

                cloud->push_back(pt);
            }

            QString cloud_id = QString("lidar_3d_cur_pts_%1").arg(idx);
            if(!viewer->updatePointCloud(cloud, cloud_id.toStdString()))
            {
                viewer->addPointCloud(cloud, cloud_id.toStdString());
            }
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id.toStdString());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, cloud_id.toStdString());

        }
    }
    else
    {
        // remove 3d lidar cloud
        for(int idx = 0; idx < config.LIDAR_3D_NUM; idx++)
        {
            QString cloud_id = QString("lidar_3d_cur_pts_%1").arg(idx);
            std::string id = cloud_id.toStdString();

            if(viewer->contains(id))
            {
                viewer->removePointCloud(id);
            }
        }
    }
}

void MainWindow::plot_mapping()
{
    if(mapping.is_mapping)
    {
        mapping.mtx.lock();

        // plot live cloud
        if(ui->ckb_PlotLive->isChecked())
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < mapping.live_cloud.pts.size(); p++)
            {
                // set pos
                pcl::PointXYZRGB pt;
                pt.x = mapping.live_cloud.pts[p].x;
                pt.y = mapping.live_cloud.pts[p].y;
                pt.z = mapping.live_cloud.pts[p].z;

                // set color
                double reflect = std::sqrt(mapping.live_cloud.pts[p].r/255);
                tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                pt.r = c.r()*255;
                pt.g = c.g()*255;
                pt.b = c.b()*255;

                // dynamic object
                if(mapping.live_cloud.pts[p].do_cnt < config.SLAM_ICP_DO_ACCUM_NUM)
                {
                    pt.r = 255;
                    pt.g = 0;
                    pt.b = 0;
                }

                cloud->push_back(pt);
            }

            if(!viewer->updatePointCloud(cloud, "live_tree_pts"))
            {
                viewer->addPointCloud(cloud, "live_tree_pts");
            }

            // point size
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, "live_tree_pts");
        }
        else
        {
            // remove realtime slam pts
            if(viewer->contains("live_tree_pts"))
            {
                viewer->removePointCloud("live_tree_pts");
            }
        }

        // plot keyframe pts
        {
            int kfrm_id;
            if(mapping.kfrm_update_que.try_pop(kfrm_id))
            {
                QString name;
                name.sprintf("kfrm_%d", kfrm_id);
                if(!viewer->contains(name.toStdString()))
                {
                    KFRAME kfrm = mapping.kfrm_storage[kfrm_id];

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    for(size_t p = 0; p < kfrm.pts.size(); p++)
                    {
                        // no drawing dynamic object
                        if(kfrm.pts[p].do_cnt < config.SLAM_ICP_DO_ACCUM_NUM)
                        {
                            continue;
                        }

                        // set pos
                        pcl::PointXYZRGB pt;
                        pt.x = kfrm.pts[p].x;
                        pt.y = kfrm.pts[p].y;
                        pt.z = kfrm.pts[p].z;

                        // set color
                        double reflect = std::sqrt(kfrm.pts[p].r/255);
                        tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                        pt.r = c.r()*255;
                        pt.g = c.g()*255;
                        pt.b = c.b()*255;

                        cloud->push_back(pt);
                    }

                    if(!viewer->updatePointCloud(cloud, name.toStdString()))
                    {
                        viewer->addPointCloud(cloud, name.toStdString());
                        viewer->updatePointCloudPose(name.toStdString(), Eigen::Affine3f(mapping.kfrm_storage[kfrm_id].opt_G.cast<float>()));
                        last_plot_kfrms.push_back(name);
                    }

                    // point size
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, name.toStdString());
                }
                else
                {
                    viewer->updatePointCloudPose(name.toStdString(), Eigen::Affine3f(mapping.kfrm_storage[kfrm_id].opt_G.cast<float>()));

                    // point size
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, name.toStdString());
                }
            }
        }

        mapping.mtx.unlock();
    }
    else
    {
        // remove first
        for(size_t p = 0; p < last_plot_kfrms.size(); p++)
        {
            QString id = last_plot_kfrms[p];
            if(viewer->contains(id.toStdString()))
            {
                viewer->removeShape(id.toStdString());
            }
        }
        last_plot_kfrms.clear();

        if(viewer->contains("live_tree_pts"))
        {
            viewer->removePointCloud("live_tree_pts");
        }
    }
}

void MainWindow::plot_loc()
{
    loc.plot_cur_pts_que.try_pop(plot_cur_pts);
    if(ui->cb_ViewType->currentText() == "VIEW_3D" && plot_cur_pts.size() > 0 && loc.is_loc && !mapping.is_mapping)
    {
        // remove first
        for(size_t p = 0; p < last_plot_kfrms.size(); p++)
        {
            QString id = last_plot_kfrms[p];
            if(viewer->contains(id.toStdString()))
            {
                viewer->removeShape(id.toStdString());
            }
        }
        last_plot_kfrms.clear();

        if(viewer->contains("live_tree_pts"))
        {
            viewer->removePointCloud("live_tree_pts");
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < plot_cur_pts.size(); p++)
        {
            Eigen::Vector3d P = plot_cur_pts[p];

            pcl::PointXYZRGB pt;
            pt.x = P[0];
            pt.y = P[1];
            pt.z = P[2];
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            cloud->push_back(pt);
        }

        if(!viewer->updatePointCloud(cloud, "plot_cur_pts"))
        {
            viewer->addPointCloud(cloud, "plot_cur_pts");
        }

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plot_cur_pts");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "plot_cur_pts");

        loc.plot_cur_pts_que.clear();
    }
    else
    {
        if(viewer->contains("plot_cur_pts"))
        {
            viewer->removePointCloud("plot_cur_pts");
        }
    }
}

void MainWindow::plot_obs()
{
    if(unimap.is_loaded != MAP_LOADED)
    {
        return;
    }

    if(is_obs_update)
    {
        is_obs_update = false;

        cv::Mat obs_map;
        cv::Mat dyn_map;
        cv::Mat vir_map;

        Eigen::Matrix4d obs_tf;
        obsmap.get_obs_map(obs_map, obs_tf);
        obsmap.get_dyn_map(dyn_map, obs_tf);
        obsmap.get_vir_map(vir_map, obs_tf);

        std::vector<Eigen::Vector4d> plot_pts = obsmap.get_plot_pts();

        // plot grid map
        {
            cv::Mat plot_obs_map(obs_map.rows, obs_map.cols, CV_8UC3, cv::Scalar(0));
            obsmap.draw_robot(plot_obs_map);

            for(int i = 0; i < obs_map.rows; i++)
            {
                for(int j = 0; j < obs_map.cols; j++)
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
        }

        // plot obs pts
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

        if(!viewer->updatePointCloud(cloud, "obs_plot_pts"))
        {
            viewer->addPointCloud(cloud, "obs_plot_pts");
        }

        // point size
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "obs_plot_pts");
    }
}

void MainWindow::plot_ctrl()
{
    // plot global path
    if(is_global_path_update)
    {
        is_global_path_update = false;

        // draw
        PATH global_path = ctrl.get_cur_global_path();
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

            if(!viewer->updatePointCloud(cloud, "global_path"))
            {
                viewer->addPointCloud(cloud, "global_path");
            }
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE*3, "global_path");
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
            if(viewer->contains(name.toStdString()))
            {
                viewer->removeShape(name.toStdString());
            }
        }
        last_plot_local_path.clear();

        // draw local path
        PATH local_path = ctrl.get_cur_local_path();

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
                    viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                    config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                    config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0, 1, 0, name.toStdString());

                    viewer->updateShapePose(name.toStdString(), Eigen::Affine3f(tf.cast<float>()));
                    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                        name.toStdString());
                    last_plot_local_path.push_back(name);
                }
            }
        }
    }

    // draw control points
    if(ctrl.is_moving)
    {
        // erase first
        if(viewer->contains("cur_pos"))
        {
            viewer->removeShape("cur_pos");
        }

        if(viewer->contains("tgt_pos"))
        {
            viewer->removeShape("tgt_pos");
        }

        if(viewer->contains("local_goal"))
        {
            viewer->removeShape("local_goal");
        }

        // draw monitoring points
        {
            Eigen::Vector3d cur_pos = ctrl.last_cur_pos;
            Eigen::Vector3d tgt_pos = ctrl.last_tgt_pos;
            Eigen::Vector3d local_goal = ctrl.last_local_goal;

            viewer->addSphere(pcl::PointXYZ(cur_pos[0], cur_pos[1], cur_pos[2]), 0.1, 1.0, 1.0, 1.0, "cur_pos");
            viewer->addSphere(pcl::PointXYZ(tgt_pos[0], tgt_pos[1], tgt_pos[2]), 0.1, 1.0, 0.0, 0.0, "tgt_pos");
            viewer->addSphere(pcl::PointXYZ(local_goal[0], local_goal[1], local_goal[2]), 0.1, 0.0, 1.0, 0.0, "local_goal");

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cur_pos");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "tgt_pos");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "local_goal");
        }
    }
    else
    {
        if(viewer->contains("cur_pos"))
        {
            viewer->removeShape("cur_pos");
        }

        if(viewer->contains("tgt_pos"))
        {
            viewer->removeShape("tgt_pos");
        }

        if(viewer->contains("local_goal"))
        {
            viewer->removeShape("local_goal");
        }
    }

    // cur tf
    Eigen::Matrix4d cur_tf = loc.get_cur_tf();

    // draw robot
    {
        // draw axis
        if(!viewer->contains("robot_axis"))
        {
            viewer->addCoordinateSystem(1.0, "robot_axis");
        }
        viewer->updateCoordinateSystemPose("robot_axis", Eigen::Affine3f(cur_tf.cast<float>()));

        // draw body
        if(!viewer->contains("robot_body"))
        {
            viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                            config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                            config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.0, 0.5, 1.0, "robot_body");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "robot_body");
        }
        viewer->updateShapePose("robot_body", Eigen::Affine3f(cur_tf.cast<float>()));

        // draw vel
        if(viewer->contains("vel_text"))
        {
            viewer->removeShape("vel_text");
        }

        pcl::PointXYZ position;
        position.x = cur_tf(0,3);
        position.y = cur_tf(1,3);
        position.z = cur_tf(2,3) + 1.5;

        // check zone
        QString zone = "";
        std::vector<QString> zones = unimap.get_nodes("ZONE");
        for(size_t p = 0; p < zones.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(zones[p]);
            if(node != NULL)
            {
                QString name = node->name;
                QString info = node->info;

                NODE_INFO res;
                if(parse_info(info, "SIZE", res))
                {
                    Eigen::Matrix4d tf = node->tf.inverse()*cur_tf;

                    double x = tf(0,3);
                    double y = tf(1,3);
                    double z = tf(2,3);

                    if(x > -res.sz[0]/2 && x < res.sz[0]/2 &&
                       y > -res.sz[1]/2 && y < res.sz[1]/2 &&
                       z > -res.sz[2]/2 && z < res.sz[2]/2)
                    {
                        // current zone
                        zone = name;
                        break;
                    }
                }
            }
        }

        QString text;
        text.sprintf("vx:%.2f, vy:%.2f, wz:%.2f, zone:%s", (double)mobile.vx0, (double)mobile.vy0, (double)mobile.wz0*R2D, zone.toLocal8Bit().data());
        ui->lb_RobotVel->setText(text);

        // draw cur node
        if(viewer->contains("cur_node"))
        {
            viewer->removeShape("cur_node");
        }

        QString cur_node_id; // = ctrl.get_cur_node_id();
        if(cur_node_id != "")
        {
            NODE *node = unimap.get_node_by_id(cur_node_id);
            if(node != NULL)
            {
                Eigen::Matrix4d tf = node->tf;
                tf(2,3) += config.ROBOT_SIZE_Z[1];
                pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS*0.5, 0.05, tf, 0.0, 1.0, 1.0);
                viewer->addPolygonMesh(donut, "cur_node");
            }
        }
    }

    // plot tactile
    {
        // erase first
        if(last_plot_tactile.size() > 0)
        {
            for(size_t p = 0; p < last_plot_tactile.size(); p++)
            {
                QString name = last_plot_tactile[p];
                if(viewer->contains(name.toStdString()))
                {
                    viewer->removeShape(name.toStdString());
                }
            }
            last_plot_tactile.clear();
        }

        //Eigen::Vector3d vel(mobile.vx0, mobile.vy0, mobile.wz0);
        Eigen::Vector3d vel = mobile.get_pose().vel;
        std::vector<Eigen::Matrix4d> traj = ctrl.calc_trajectory(vel, 0.2, 1.0, cur_tf);
        for(size_t p = 0; p < traj.size(); p++)
        {
            QString name;
            name.sprintf("traj_%d", (int)p);

            viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                    config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                    config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 1.0, 0.0, 0.0, name.toStdString());

            viewer->updateShapePose(name.toStdString(), Eigen::Affine3f(traj[p].cast<float>()));

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                name.toStdString());
            last_plot_tactile.push_back(name);
        }
    }

    // plot goal info
    ui->lb_RobotGoal->setText(QString("id:%1\ninfo:%2").arg(ctrl.move_info.goal_node_id).arg(ctrl.cur_goal_state));
}

void MainWindow::plot_loop()
{
    plot_timer.stop();
    double st_time = get_time();

    plot_map();
    plot_topo();
    plot_pick();
    plot_info();
    plot_raw_2d();
    plot_raw_3d();
    plot_mapping();
    plot_loc();
    plot_obs();
    plot_ctrl();

    // camera reset
    if(is_set_top_view)
    {
        is_set_top_view = false;
        viewer->setCameraPosition(0, 0, 30, 0, 0, 0, 0, 1, 0);
        printf("[MAIN] set top view camera\n");
    }

    if(is_view_reset)
    {
        is_view_reset = false;
        viewer->resetCamera();
        printf("[MAIN] reset view camera\n");
    }

    // mapping view
    if(ui->cb_ViewType->currentText() == "VIEW_MAPPING")
    {
        Eigen::Matrix4d cur_tf = loc.get_cur_tf();
        Eigen::Matrix4d _cur_tf = se2_to_TF(TF_to_se2(cur_tf));
        _cur_tf(2,3) = cur_tf(2,3);

        double d = ui->cb_ViewHeight->currentText().toDouble();
        Eigen::Vector3d cam_pos = _cur_tf.block(0,0,3,3)*Eigen::Vector3d(0,0,d) + _cur_tf.block(0,3,3,1);
        Eigen::Vector3d cam_tgt = _cur_tf.block(0,0,3,3)*Eigen::Vector3d(0,0,0) + _cur_tf.block(0,3,3,1);
        Eigen::Vector3d cam_up(1,0,0);

        viewer->setCameraPosition(cam_pos[0], cam_pos[1], cam_pos[2],
                                  cam_tgt[0], cam_tgt[1], cam_tgt[2],
                                  cam_up[0], cam_up[1], cam_up[2]);

        double near = 2.0;
        double far = 2.0*d;
        viewer->setCameraClipDistances(near, far);
    }

    // rendering
    ui->qvtkWidget->renderWindow()->Render();

    // check plot drift
    plot_proc_t = get_time() - st_time;
    if(plot_proc_t > 0.1)
    {
        //printf("[MAIN] plot_loop, loop time drift: %f\n", (double)plot_proc_t);
    }

    plot_timer.start();
}
