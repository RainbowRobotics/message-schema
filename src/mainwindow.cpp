#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , config(this)
    , logger(this)
    , unimap(this)
    , lidar_3d(this)
    , plot_timer(this)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // mapping
    connect(ui->bt_MapLoad, SIGNAL(clicked()), this, SLOT(bt_MapLoad()));

    // for 3d viewer
    connect(ui->cb_ViewType, SIGNAL(currentIndexChanged(QString)), this, SLOT(all_update()));

    // for simulation
    connect(ui->bt_SimInit, SIGNAL(clicked()), this, SLOT(bt_SimInit()));

    // config
    connect(ui->bt_ConfigLoad, SIGNAL(clicked()), this, SLOT(bt_ConfigLoad()));

    // emergency stop
    connect(ui->bt_Emergency, SIGNAL(clicked()), this, SLOT(bt_Emergency()));

    // motor
    connect(ui->bt_MotorInit, SIGNAL(clicked()), this, SLOT(bt_MotorInit()));

    connect(ui->bt_Sync, SIGNAL(clicked()), this, SLOT(bt_Sync()));

    // localization
    connect(ui->bt_LocInit, SIGNAL(clicked()), this, SLOT(bt_LocInit()));
    connect(ui->bt_LocStart, SIGNAL(clicked()), this, SLOT(bt_LocStart()));
    connect(ui->bt_LocStop, SIGNAL(clicked()), this, SLOT(bt_LocStop()));

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
    config.config_path = QCoreApplication::applicationDirPath() + "/../configs/" + QString(ROBOT_PLATFORM) + "/config.json";
    config.load();

    // simulation check
    if(config.USE_SIM)
    {
        this->setWindowTitle("SLAMNAV2 (SIMULATION MODE)");

        QPalette palette = this->palette();
        palette.setColor(QPalette::Window, Qt::red);
        this->setPalette(palette);

        ui->bt_SimInit->setEnabled(true);
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

    // lidar 3d module init
    if(config.USE_LIDAR_3D)
    {
        lidar_3d.config = &config;
        lidar_3d.logger = &logger;
        lidar_3d.init();
        lidar_3d.open();
        // lidar_3d.mobile = &mobile;
    }

    // localization module init
    loc.config = &config;
    loc.logger = &logger;
    loc.mobile = &mobile;
    loc.lidar_3d = &lidar_3d;
    // loc.cam = &cam;
    loc.unimap = &unimap;
    loc.obsmap = &obsmap;

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

    // start jog loop
    jog_flag = true;
    jog_thread = new std::thread(&MainWindow::jog_loop, this);

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

void MainWindow::bt_MapLoad()
{
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    if(!path.isNull())
    {
        // slam.localization_stop();
        // obsmap.clear();

        config.set_map_path(path);

        // map_dir = path;
        unimap.load_map(path);
        all_update();
    }
}

// for mobile platform
void MainWindow::bt_SimInit()
{

}

void MainWindow::bt_ConfigLoad()
{
    config.load();
}

// for emergency stop
void MainWindow::bt_Emergency()
{
    // task.cancel();
    // ctrl.stop();
    // ctrl.set_multi_req("none");
    // ctrl.set_obs_condition("none");
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
    // lidar.sync_f();
    // lidar.sync_b();

    // if(config.USE_LVX)
    // {
    //     lvx.is_sync = true;
    // }
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

// for plot loop
void MainWindow::map_plot()
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

void MainWindow::topo_plot()
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

void MainWindow::pick_plot()
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

void MainWindow::info_plot()
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

    // 3d lidar
    if(config.USE_LIDAR_3D)
    {
        ui->lb_LidarInfo_3D->setText(lidar_3d.get_info_text());
    }

}

void MainWindow::raw_plot()
{
    // plot 3d lidar
    if(lidar_3d.is_connected && ui->cb_ViewType->currentText() == "VIEW_3D" && !loc.is_loc)
    {
        for(int idx = 0; idx < config.LIDAR_3D_NUM; idx++)
        {
            // plot current lidar data
            LVX_FRM cur_frm = lidar_3d.get_cur_frm(idx);

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

            QString cloud_id = QString("lidar_cur_pts_%1").arg(idx);
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
            QString cloud_id = QString("lidar_cur_pts_%1").arg(idx);
            std::string id = cloud_id.toStdString();

            if(viewer->contains(id))
            {
                viewer->removePointCloud(id);
            }
        }
    }
}

void MainWindow::loc_plot()
{
    loc.plot_cur_pts_que.try_pop(plot_cur_pts);
    if(ui->cb_ViewType->currentText() == "VIEW_3D" && plot_cur_pts.size() > 0)
    {
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

void MainWindow::ctrl_plot()
{
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

}

void MainWindow::plot_loop()
{
    plot_timer.stop();
    double st_time = get_time();

    map_plot();
    topo_plot();
    pick_plot();
    info_plot();
    raw_plot();
    loc_plot();
    ctrl_plot();

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
