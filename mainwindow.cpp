#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , config(this)
    , mobile(this)
    , lidar(this)
    , cam(this)
    , code(this)
    , slam(this)
    , unimap(this)
    , obsmap(this)
    , ctrl(this)    
    , ws(this)
    , sim(this)
    , ui(new Ui::MainWindow)
    , plot_timer(this)
    , plot_timer2(this)
    , watchdog_timer(this)
    , qa_timer(this)
{
    ui->setupUi(this);

    // for tab
    connect(ui->main_tab, SIGNAL(currentChanged(int)), this, SLOT(all_update()));
    connect(ui->annot_tab, SIGNAL(currentChanged(int)), this, SLOT(all_update()));

    // timer
    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    connect(&plot_timer2, SIGNAL(timeout()), this, SLOT(plot_loop2()));
    connect(&watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdog_loop()));
    connect(&qa_timer, SIGNAL(timeout()), this, SLOT(qa_loop()));

    // config
    connect(ui->bt_ConfigLoad, SIGNAL(clicked()), this, SLOT(bt_ConfigLoad()));
    connect(ui->bt_ConfigSave, SIGNAL(clicked()), this, SLOT(bt_ConfigSave()));

    // motor
    connect(ui->bt_MotorInit, SIGNAL(clicked()), this, SLOT(bt_MotorInit()));

    // test
    connect(ui->bt_Sync, SIGNAL(clicked()), this, SLOT(bt_Sync()));
    connect(ui->bt_MoveLinear, SIGNAL(clicked()), this, SLOT(bt_MoveLinear()));
    connect(ui->bt_MoveRotate, SIGNAL(clicked()), this, SLOT(bt_MoveRotate()));

    // jog
    connect(ui->bt_JogF, SIGNAL(pressed()), this, SLOT(bt_JogF()));
    connect(ui->bt_JogB, SIGNAL(pressed()), this, SLOT(bt_JogB()));
    connect(ui->bt_JogL, SIGNAL(pressed()), this, SLOT(bt_JogL()));
    connect(ui->bt_JogR, SIGNAL(pressed()), this, SLOT(bt_JogR()));
    connect(ui->bt_JogF, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogB, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogL, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogR, SIGNAL(released()), this, SLOT(bt_JogReleased()));

    // mapping
    connect(ui->bt_MapBuild, SIGNAL(clicked()), this, SLOT(bt_MapBuild()));    
    connect(ui->bt_MapSave, SIGNAL(clicked()), this, SLOT(bt_MapSave()));
    connect(ui->bt_MapLoad, SIGNAL(clicked()), this, SLOT(bt_MapLoad()));

    // localization
    connect(ui->bt_LocInit, SIGNAL(clicked()), this, SLOT(bt_LocInit()));
    connect(ui->bt_LocInit2, SIGNAL(clicked()), this, SLOT(bt_LocInit2()));
    connect(ui->bt_LocStart, SIGNAL(clicked()), this, SLOT(bt_LocStart()));
    connect(ui->bt_LocStop, SIGNAL(clicked()), this, SLOT(bt_LocStop()));

    // annotation
    connect(ui->bt_MapLoad2, SIGNAL(clicked()), this, SLOT(bt_MapLoad()));
    connect(ui->bt_MapReload, SIGNAL(clicked()), this, SLOT(bt_MapReload()));
    connect(ui->bt_MapSave2, SIGNAL(clicked()), this, SLOT(bt_MapSave2()));
    connect(ui->bt_AddNode, SIGNAL(clicked()), this, SLOT(bt_AddNode()));
    connect(ui->bt_AddLink1, SIGNAL(clicked()), this, SLOT(bt_AddLink1()));
    connect(ui->bt_AddLink2, SIGNAL(clicked()), this, SLOT(bt_AddLink2()));
    connect(ui->bt_EditNodePos, SIGNAL(clicked()), this, SLOT(bt_EditNodePos()));
    connect(ui->bt_EditNodeType, SIGNAL(clicked()), this, SLOT(bt_EditNodeType()));
    connect(ui->bt_EditNodeInfo, SIGNAL(clicked()), this, SLOT(bt_EditNodeInfo()));
    connect(ui->bt_MinGapNodeX, SIGNAL(clicked()), this, SLOT(bt_MinGapNodeX()));
    connect(ui->bt_MinGapNodeY, SIGNAL(clicked()), this, SLOT(bt_MinGapNodeY()));
    connect(ui->bt_AlignNodeX, SIGNAL(clicked()), this, SLOT(bt_AlignNodeX()));
    connect(ui->bt_AlignNodeY, SIGNAL(clicked()), this, SLOT(bt_AlignNodeY()));
    connect(ui->bt_AlignNodeTh, SIGNAL(clicked()), this, SLOT(bt_AlignNodeTh()));
    connect(ui->bt_ClearTopo, SIGNAL(clicked()), this, SLOT(bt_ClearTopo()));

    connect(ui->bt_NodePoseXUp, SIGNAL(clicked()), this, SLOT(bt_NodePoseXUp()));
    connect(ui->bt_NodePoseYUp, SIGNAL(clicked()), this, SLOT(bt_NodePoseYUp()));
    connect(ui->bt_NodePoseThUp, SIGNAL(clicked()), this, SLOT(bt_NodePoseThUp()));
    connect(ui->bt_NodePoseXDown, SIGNAL(clicked()), this, SLOT(bt_NodePoseXDown()));
    connect(ui->bt_NodePoseYDown, SIGNAL(clicked()), this, SLOT(bt_NodePoseYDown()));
    connect(ui->bt_NodePoseThDown, SIGNAL(clicked()), this, SLOT(bt_NodePoseThDown()));

    connect(ui->bt_QuickAnnotStart, SIGNAL(clicked()), this, SLOT(bt_QuickAnnotStart()));
    connect(ui->bt_QuickAnnotStop, SIGNAL(clicked()), this, SLOT(bt_QuickAnnotStop()));

    // for simulation
    connect(ui->bt_SimInit, SIGNAL(clicked()), this, SLOT(bt_SimInit()));

    // for autocontrol
    connect(ui->bt_AutoMove, SIGNAL(clicked()), this, SLOT(bt_AutoMove()));
    connect(ui->bt_AutoMove2, SIGNAL(clicked()), this, SLOT(bt_AutoMove2()));
    connect(ui->bt_AutoMove3, SIGNAL(clicked()), this, SLOT(bt_AutoMove3()));
    connect(ui->bt_AutoStop, SIGNAL(clicked()), this, SLOT(bt_AutoStop()));

    // for ws
    connect(&ws, SIGNAL(recv_command_motorinit(double)), this, SLOT(ws_command_motorinit(double)));
    connect(&ws, SIGNAL(recv_command_move(double, double, double, double)), this, SLOT(ws_command_move(double, double, double, double)));

    // for obsmap
    connect(&obsmap, SIGNAL(obs_updated()), this, SLOT(obs_update()));
    connect(ui->bt_ObsClear, SIGNAL(clicked()), this, SLOT(bt_ObsClear()));

    // set plot window
    setup_vtk();

    // init modules
    init_modules();

    // start plot loop
    plot_timer.start(50);
    plot_timer2.start(50);
    watchdog_timer.start(100);

    // solve tab with vtk render window problem
    QTimer::singleShot(100, [&]()
    {
        ui->main_tab->setCurrentIndex(1);
        QTimer::singleShot(100, [&]()
        {
            ui->main_tab->setCurrentIndex(0);
        });
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

// for replot
void MainWindow::map_update()
{
    is_map_update = true;
    is_map_update2 = true;
}

void MainWindow::topo_update()
{
    is_topo_update = true;
    is_topo_update2 = true;
}

void MainWindow::pick_update()
{
    is_pick_update = true;
    is_pick_update2 = true;
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

// for init
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

        // init drawing
        viewer->addCoordinateSystem(1.0, "O_global");
        ui->qvtkWidget->renderWindow()->Render();

        // install event filter
        ui->qvtkWidget->installEventFilter(this);
        ui->qvtkWidget->setMouseTracking(true);
    }

    // Set up the QVTK window 2
    {
        vtkNew<vtkGenericOpenGLRenderWindow> renderWindow2;
        vtkNew<vtkRenderer> renderer2;
        renderWindow2->AddRenderer(renderer2);
        viewer2.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer2", false));
        ui->qvtkWidget2->setRenderWindow(viewer2->getRenderWindow());
        viewer2->setupInteractor(ui->qvtkWidget2->interactor(), ui->qvtkWidget2->renderWindow());
        viewer2->setBackgroundColor(1.0, 1.0, 1.0);
        viewer2->resetCamera();

        // init drawing
        viewer2->addCoordinateSystem(1.0, "O_global");
        ui->qvtkWidget2->renderWindow()->Render();

        // install event filter
        ui->qvtkWidget2->installEventFilter(this);
        ui->qvtkWidget2->setMouseTracking(true);
    }
}

void MainWindow::init_modules()
{
    // config module init
    config.config_path = QCoreApplication::applicationDirPath() + "/config.json";
    config.ui_table = ui->tw_Config;
    config.load();
    config.config_to_ui();

    if(config.SIM_MODE == 1)
    {
        this->setWindowTitle("SLAMNAV2 (SIMULATION MODE)");

        QPalette palette = this->palette();
        palette.setColor(QPalette::Window, Qt::red);
        this->setPalette(palette);

        ui->bt_SimInit->setEnabled(true);
    }

    // log module init
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

    // lidar module init
    lidar.config = &config;
    lidar.logger = &logger;
    lidar.mobile = &mobile;
    lidar.open();

    // cam module init
    cam.config = &config;
    cam.logger = &logger;
    cam.mobile = &mobile;
    cam.init();

    // code reader module init
    code.config = &config;
    code.logger = &logger;
    code.unimap = &unimap;
    code.init();

    // slam module init
    slam.config = &config;
    slam.logger = &logger;
    slam.mobile = &mobile;
    slam.lidar = &lidar;
    slam.cam = &cam;
    slam.unimap = &unimap;
    slam.obsmap = &obsmap;

    // autocontrol module init
    ctrl.config = &config;
    ctrl.logger = &logger;
    ctrl.mobile = &mobile;
    ctrl.lidar = &lidar;
    ctrl.cam = &cam;
    ctrl.code = &code;
    ctrl.slam = &slam;
    ctrl.unimap = &unimap;
    ctrl.obsmap = &obsmap;
    ctrl.init();

    // websocket client init
    ws.config = &config;
    ws.logger = &logger;
    ws.mobile = &mobile;
    ws.lidar = &lidar;
    ws.cam = &cam;
    ws.code = &code;
    ws.slam = &slam;
    ws.unimap = &unimap;
    ws.obsmap = &obsmap;
    ws.ctrl = &ctrl;
    ws.init();

    // simulation module init
    sim.config = &config;
    sim.logger = &logger;
    sim.mobile = &mobile;
    sim.lidar = &lidar;
    sim.slam = &slam;
    sim.unimap = &unimap;
}

// for picking interface
bool MainWindow::eventFilter(QObject *object, QEvent *ev)
{
    if(object == ui->qvtkWidget)
    {
        // keyboard event
        if(ev->type() == QEvent::KeyPress)
        {
            QKeyEvent* ke = static_cast<QKeyEvent*>(ev);
            switch(ke->key())
            {
                case Qt::Key_Up:
                    mobile.move(ui->spb_JogV->value(), 0, mobile.wz0);
                    break;
                case Qt::Key_Left:
                    mobile.move(mobile.vx0, 0, ui->spb_JogW->value()*D2R);
                    break;
                case Qt::Key_Down:
                    mobile.move(-ui->spb_JogV->value(), 0, mobile.wz0);
                    break;
                case Qt::Key_Right:
                    mobile.move(mobile.vx0, 0, -ui->spb_JogW->value()*D2R);
                    break;
                default:
                    return false;
            }
            return true;
        }
        else if(ev->type() == QEvent::KeyRelease)
        {
            QKeyEvent* ke = static_cast<QKeyEvent*>(ev);
            switch(ke->key())
            {
                case Qt::Key_Up:
                    mobile.move(0, 0, mobile.wz0);
                    break;
                case Qt::Key_Left:
                    mobile.move(mobile.vx0, 0, 0);
                    break;
                case Qt::Key_Down:
                    mobile.move(0, 0, mobile.wz0);
                    break;
                case Qt::Key_Right:
                    mobile.move(mobile.vx0, 0, 0);
                    break;
                default:
                    return false;
            }
            return true;
        }

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
                pick.cur_node = unimap.get_node_id(pt);

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
    else if(object == ui->qvtkWidget2)
    {
        // keyboard event
        if(ev->type() == QEvent::KeyPress)
        {
            QKeyEvent* ke = static_cast<QKeyEvent*>(ev);
            switch(ke->key())
            {
                case Qt::Key_Up:
                    mobile.move(ui->spb_JogV->value(), 0, mobile.wz0);
                    break;
                case Qt::Key_Left:
                    mobile.move(mobile.vx0, 0, ui->spb_JogW->value()*D2R);
                    break;
                case Qt::Key_Down:
                    mobile.move(-ui->spb_JogV->value(), 0, mobile.wz0);
                    break;
                case Qt::Key_Right:
                    mobile.move(mobile.vx0, 0, -ui->spb_JogW->value()*D2R);
                    break;
                default:
                    return false;
            }
            return true;
        }
        else if(ev->type() == QEvent::KeyRelease)
        {
            QKeyEvent* ke = static_cast<QKeyEvent*>(ev);
            switch(ke->key())
            {
                case Qt::Key_Up:
                    mobile.move(0, 0, mobile.wz0);
                    break;
                case Qt::Key_Left:
                    mobile.move(mobile.vx0, 0, 0);
                    break;
                case Qt::Key_Down:
                    mobile.move(0, 0, mobile.wz0);
                    break;
                case Qt::Key_Right:
                    mobile.move(mobile.vx0, 0, 0);
                    break;
                default:
                    return false;
            }
            return true;
        }

        // different behavior each tab
        if(ui->annot_tab->tabText(ui->annot_tab->currentIndex()) == "EDIT_MAP")
        {
            // mouse event
            if(ev->type() == QEvent::MouseButtonPress)
            {
                QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                if(me->button() == Qt::LeftButton)
                {
                    // clear node selection
                    pick.pre_node = "";
                    pick.cur_node = "";

                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.l_pt0 = pt;
                    pick.l_drag = true;

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
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt0 = pt;
                    pick.r_drag = true;

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
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.l_pt0 = pt;

                    pick_update();
                    return true;
                }

                if(pick.r_drag)
                {
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt0 = pt;

                    pick_update();
                    return true;
                }
            }

            if(ev->type() == QEvent::MouseButtonRelease)
            {
                QMouseEvent* me = static_cast<QMouseEvent*>(ev);
                if(me->button() == Qt::LeftButton)
                {
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.l_pt0 = pt;
                    pick.l_drag = false;

                    pick_update();
                    return true;
                }

                if(me->button() == Qt::RightButton)
                {
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt0 = pt;
                    pick.r_drag = false;

                    pick_update();
                    return true;
                }
            }
        }
        else if(ui->annot_tab->tabText(ui->annot_tab->currentIndex()) == "EDIT_TOPO")
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
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.pre_node = pick.cur_node;
                    pick.cur_node = unimap.get_node_id(pt);

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
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

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
                    return true;
                }

                if(pick.r_drag)
                {
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

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
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

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

    return QWidget::eventFilter(object, ev);
}

void MainWindow::picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer)
{
    // ray casting
    std::vector<pcl::visualization::Camera> cams;
    pcl_viewer->getCameras(cams);

    Eigen::Matrix4d proj;
    Eigen::Matrix4d view;
    cams[0].computeProjectionMatrix(proj);
    cams[0].computeViewMatrix(view);

    Eigen::Vector3d pos(cams[0].pos[0], cams[0].pos[1], cams[0].pos[2]);

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

// for mobile platform
void MainWindow::bt_SimInit()
{
    if(unimap.is_loaded == false)
    {
        printf("[SIM] map load first\n");
        return;
    }

    // stop first
    if(slam.is_loc)
    {
        slam.localization_stop();
    }
    sim.stop();

    // start
    sim.start();

    mobile.is_connected = true;
    mobile.is_synced = true;

    lidar.is_connected_f = true;
    lidar.is_connected_b = true;

    lidar.is_synced_f = true;
    lidar.is_synced_b = true;
}

void MainWindow::bt_ConfigLoad()
{
    config.load();
    config.config_to_ui();
}

void MainWindow::bt_ConfigSave()
{
    config.save();

    // reload
    config.load();
    config.config_to_ui();
}

void MainWindow::bt_MotorInit()
{
    mobile.motor_on();
}

void MainWindow::bt_Sync()
{
    mobile.sync();
    lidar.sync_f();
    lidar.sync_b();
}

void MainWindow::bt_MoveLinear()
{
    double d = ui->dsb_MoveLinearDist->value();
    double v = ui->dsb_MoveLinearV->value();

    mobile.move_linear(d, v);
}

void MainWindow::bt_MoveRotate()
{
    double th = ui->dsb_MoveRotateDeg->value() * D2R;
    double w = ui->dsb_MoveRotateW->value() * D2R;

    mobile.move_rotate(th, w);
}

void MainWindow::bt_JogF()
{
    double vx = ui->spb_JogV->value();
    double vy = 0;
    double wz = 0;

    mobile.move(vx, vy, wz);
    printf("[JOG] %f, %f, %f\n", vx, vy, wz*R2D);
}

void MainWindow::bt_JogB()
{
    double vx = -ui->spb_JogV->value();
    double vy = 0;
    double wz = 0;

    mobile.move(vx, vy, wz);
    printf("[JOG] %f, %f, %f\n", vx, vy, wz*R2D);
}

void MainWindow::bt_JogL()
{
    double vx = 0;
    double vy = 0;
    double wz = ui->spb_JogW->value()*D2R;

    mobile.move(vx, vy, wz);
    printf("[JOG] %f, %f, %f\n", vx, vy, wz*R2D);
}

void MainWindow::bt_JogR()
{
    double vx = 0;
    double vy = 0;
    double wz = -ui->spb_JogW->value()*D2R;

    mobile.move(vx, vy, wz);
    printf("[JOG] %f, %f, %f\n", vx, vy, wz*R2D);
}

void MainWindow::bt_JogReleased()
{
    mobile.move(0, 0, 0);
    printf("[JOG] 0, 0, 0\n");
}

// for mapping and localization
void MainWindow::bt_MapBuild()
{
    if(config.SIM_MODE == 1)
    {
        printf("[MAIN] map build not allowed SIM_MODE\n");
        return;
    }

    // auto generation dir path
    QString _map_dir = QDir::homePath() + "/maps/" + get_time_str();
    QDir().mkpath(_map_dir);
    map_dir = _map_dir;

    // mapping start
    slam.mapping_start();
}

void MainWindow::bt_MapSave()
{
    if(config.SIM_MODE == 1)
    {
        printf("[MAIN] map save not allowed SIM_MODE\n");
        return;
    }

    // check
    if(map_dir == "")
    {
        printf("[MAIN] no map_dir\n");
        return;
    }

    // stop first
    slam.mapping_stop();

    // check kfrm
    if(slam.kfrm_storage.size() == 0)
    {
        printf("[MAIN] no keyframe\n");
        return;
    }

    // get all points and sampling
    const uint64_t p1 = 73856093;
    const uint64_t p2 = 19349669;
    const uint64_t p3 = 83492791;
    const double voxel_size = config.SLAM_VOXEL_SIZE;

    std::unordered_map<uint64_t, uint8_t> hash_map;
    std::vector<PT_XYZR> pts;
    for(size_t p = 0; p < slam.kfrm_storage.size(); p++)
    {
        Eigen::Matrix4d G = slam.kfrm_storage[p].opt_G;
        for(size_t q = 0; q < slam.kfrm_storage[p].pts.size(); q++)
        {
            Eigen::Vector3d P;
            P[0] = slam.kfrm_storage[p].pts[q].x;
            P[1] = slam.kfrm_storage[p].pts[q].y;
            P[2] = slam.kfrm_storage[p].pts[q].z;

            Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);

            uint64_t x = _P[0]/voxel_size;
            uint64_t y = _P[1]/voxel_size;
            uint64_t z = _P[2]/voxel_size;
            uint64_t key = x*p1 ^ y*p2 ^ z*p3; // unlimited bucket size
            if(hash_map.find(key) == hash_map.end())
            {
                hash_map[key] = 1;

                PT_XYZR pt;
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                pt.r = slam.kfrm_storage[p].pts[q].r;
                pts.push_back(pt);
            }
        }

        printf("[MAIN] convert: %d/%d ..\n", (int)(p+1), (int)slam.kfrm_storage.size());
    }

    // write file
    QString cloud_csv_path = map_dir + "/cloud.csv";
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
        map_dir = path;
        unimap.load_map(path);
        all_update();
    }
}

void MainWindow::bt_LocInit()
{
    // manual init
    slam.mtx.lock();
    slam.cur_tf = se2_to_TF(pick.r_pose);
    slam.mtx.unlock();
}

void MainWindow::bt_LocInit2()
{
    // auto init
}

void MainWindow::bt_LocStart()
{
    slam.localization_start();
}

void MainWindow::bt_LocStop()
{
    slam.localization_stop();
}

// for annotation
void MainWindow::bt_MapSave2()
{
    unimap.save_map();
    unimap.save_annotation();
}

void MainWindow::bt_MapReload()
{
    if(unimap.is_loaded == false)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    unimap.load_map(map_dir);
    all_update();
}

void MainWindow::bt_AddNode()
{
    unimap.add_node(pick, ui->cb_NodeType->currentText());    
    topo_update();
}

void MainWindow::bt_EditNodePos()
{
    unimap.edit_node_pos(pick);
    topo_update();
}

void MainWindow::bt_EditNodeType()
{
    unimap.edit_node_type(pick, ui->cb_NodeType->currentText());
    topo_update();
}

void MainWindow::bt_EditNodeInfo()
{
    unimap.edit_node_info(pick, ui->te_NodeInfo->toPlainText());
    topo_update();
}

void MainWindow::bt_AddLink1()
{
    unimap.add_link1(pick);
    topo_update();
}

void MainWindow::bt_AddLink2()
{
    unimap.add_link2(pick);
    topo_update();
}

void MainWindow::bt_ClearTopo()
{
    if(unimap.is_loaded == false)
    {
        return;
    }

    unimap.clear_nodes();
    topo_update();
}

void MainWindow::bt_NodePoseXUp()
{
    QString id = pick.cur_node;
    if(id == "")
    {
        return;
    }

    double dx = ui->spb_NodeMovementXY->value();
    NODE* node = unimap.get_node_by_id(id);
    node->tf(0,3) += dx;

    ui->le_CurNodeX->setText(QString::number(node->tf(0,3)));
    topo_update();
}

void MainWindow::bt_NodePoseYUp()
{
    QString id = pick.cur_node;
    if(id == "")
    {
        return;
    }

    double dy = ui->spb_NodeMovementXY->value();
    NODE* node = unimap.get_node_by_id(id);
    node->tf(1,3) += dy;

    ui->le_CurNodeY->setText(QString::number(node->tf(1,3)));
    topo_update();
}

void MainWindow::bt_NodePoseThUp()
{
    QString id = pick.cur_node;
    if(id == "")
    {
        return;
    }

    NODE* node = unimap.get_node_by_id(id);
    Eigen::Matrix4d tf0 = node->tf;

    double dth = ui->spb_NodeMovementTh->value()*D2R;
    Eigen::Matrix4d tf = ZYX_to_TF(0, 0, 0, 0, 0, dth);

    node->tf = tf0 * tf;

    Eigen::Matrix3d R = node->tf.block(0,0,3,3);
    Eigen::Vector3d euler = R.eulerAngles(2,1,0);
    double th = euler[0];
    ui->le_CurNodeTh->setText(QString::number(th*R2D));
    topo_update();
}

void MainWindow::bt_NodePoseXDown()
{
    QString id = pick.cur_node;
    if(id == "")
    {
        return;
    }

    double dx = ui->spb_NodeMovementXY->value();
    NODE* node = unimap.get_node_by_id(id);
    node->tf(0,3) -= dx;

    ui->le_CurNodeX->setText(QString::number(node->tf(0,3)));
    topo_update();
}

void MainWindow::bt_NodePoseYDown()
{
    QString id = pick.cur_node;
    if(id == "")
    {
        return;
    }

    double dy = ui->spb_NodeMovementXY->value();
    NODE* node = unimap.get_node_by_id(id);
    node->tf(1,3) -= dy;

    ui->le_CurNodeY->setText(QString::number(node->tf(1,3)));
    topo_update();
}

void MainWindow::bt_NodePoseThDown()
{
    QString id = pick.cur_node;
    if(id == "")
    {
        return;
    }

    NODE* node = unimap.get_node_by_id(id);
    Eigen::Matrix4d tf0 = node->tf;

    double dth = -ui->spb_NodeMovementTh->value();
    Eigen::Matrix4d tf = ZYX_to_TF(0, 0, 0, 0, 0, dth*D2R);

    node->tf = tf0 * tf;

    Eigen::Matrix3d R = node->tf.block(0,0,3,3);
    Eigen::Vector3d euler = R.eulerAngles(2,1,0);
    double th = euler[0];
    ui->le_CurNodeTh->setText(QString::number(th*R2D));
    topo_update();
}

void MainWindow::bt_MinGapNodeX()
{
    double gap = ui->spb_NodeGap->value();

    QString id0 = pick.pre_node;
    QString id1 = pick.cur_node;
    if(id0 == "" || id1 == "")
    {
        return;
    }

    Eigen::Matrix4d tf0 = unimap.get_node_by_id(id0)->tf;
    Eigen::Matrix4d tf1 = unimap.get_node_by_id(id1)->tf;

    if(tf0(0,3) > tf1(0,3))
    {
        tf1(0,3) = tf0(0,3) + gap;
    }
    else
    {
        tf1(0,3) = tf0(0,3) - gap;
    }

    unimap.edit_node_pos(id1, tf1);
    topo_update();
}

void MainWindow::bt_MinGapNodeY()
{
    double gap = ui->spb_NodeGap->value();

    QString id0 = pick.pre_node;
    QString id1 = pick.cur_node;
    if(id0 == "" || id1 == "")
    {
        return;
    }

    Eigen::Matrix4d tf0 = unimap.get_node_by_id(id0)->tf;
    Eigen::Matrix4d tf1 = unimap.get_node_by_id(id1)->tf;

    if(tf0(1,3) > tf1(1,3))
    {
        tf1(1,3) = tf0(1,3) + gap;
    }
    else
    {
        tf1(1,3) = tf0(1,3) - gap;
    }

    unimap.edit_node_pos(id1, tf1);
    topo_update();
}

void MainWindow::bt_MinGapNodeZ()
{
    double gap = ui->spb_NodeGap->value();

    QString id0 = pick.pre_node;
    QString id1 = pick.cur_node;
    if(id0 == "" || id1 == "")
    {
        return;
    }

    Eigen::Matrix4d tf0 = unimap.get_node_by_id(id0)->tf;
    Eigen::Matrix4d tf1 = unimap.get_node_by_id(id1)->tf;

    if(tf0(2,3) > tf1(2,3))
    {
        tf1(2,3) = tf0(2,3) + gap;
    }
    else
    {
        tf1(2,3) = tf0(2,3) - gap;
    }

    unimap.edit_node_pos(id1, tf1);
    topo_update();
}

void MainWindow::bt_AlignNodeX()
{
    QString id0 = pick.pre_node;
    QString id1 = pick.cur_node;
    if(id0 == "" || id1 == "")
    {
        return;
    }

    Eigen::Matrix4d tf0 = unimap.get_node_by_id(id0)->tf;
    Eigen::Matrix4d tf1 = unimap.get_node_by_id(id1)->tf;

    // align x
    tf1(0,3) = tf0(0,3);
    unimap.edit_node_pos(id1, tf1);
    topo_update();
}

void MainWindow::bt_AlignNodeY()
{
    QString id0 = pick.pre_node;
    QString id1 = pick.cur_node;
    if(id0 == "" || id1 == "")
    {
        return;
    }

    Eigen::Matrix4d tf0 = unimap.get_node_by_id(id0)->tf;
    Eigen::Matrix4d tf1 = unimap.get_node_by_id(id1)->tf;

    // align x
    tf1(1,3) = tf0(1,3);
    unimap.edit_node_pos(id1, tf1);
    topo_update();
}

void MainWindow::bt_AlignNodeZ()
{
    QString id0 = pick.pre_node;
    QString id1 = pick.cur_node;
    if(id0 == "" || id1 == "")
    {
        return;
    }

    Eigen::Matrix4d tf0 = unimap.get_node_by_id(id0)->tf;
    Eigen::Matrix4d tf1 = unimap.get_node_by_id(id1)->tf;

    // align x
    tf1(2,3) = tf0(2,3);
    unimap.edit_node_pos(id1, tf1);
    topo_update();
}

void MainWindow::bt_AlignNodeTh()
{
    QString id0 = pick.pre_node;
    QString id1 = pick.cur_node;
    if(id0 == "" || id1 == "")
    {
        return;
    }

    Eigen::Matrix4d tf0 = unimap.get_node_by_id(id0)->tf;
    Eigen::Matrix4d tf1 = unimap.get_node_by_id(id1)->tf;

    // align x
    tf1.block(0,0,3,3) = tf0.block(0,0,3,3);
    unimap.edit_node_pos(id1, tf1);
    topo_update();
}

void MainWindow::bt_QuickAnnotStart()
{
    if(unimap.is_loaded == false)
    {
        printf("[QA] check map load\n");
        return;
    }

    if(slam.is_loc == false)
    {
        printf("[QA] check localization\n");
        return;
    }

    if(qa_timer.isActive())
    {
        printf("[QA] qa already running\n");
        return;
    }


    Eigen::Matrix4d cur_tf = slam.get_cur_tf();
    qa_last_node = unimap.add_node(cur_tf, "GOAL");

    qa_timer.start(1000);

    topo_update();
    printf("[QA] qa start\n");
}

void MainWindow::bt_QuickAnnotStop()
{
    if(!qa_timer.isActive())
    {
        return;
    }

    // loop stop
    qa_timer.stop();

    NODE* last_node = unimap.get_node_by_id(qa_last_node);
    if(last_node == NULL)
    {
        printf("[QA] last node empty\n");
        return;
    }

    Eigen::Matrix4d cur_tf = slam.get_cur_tf();

    double d = (cur_tf.block(0,3,3,1) - last_node->tf.block(0,3,3,1)).norm();
    if(d > config.ANNOT_QA_STEP)
    {
        QString cur_node = unimap.add_node(cur_tf, "GOAL"); // final node
        unimap.add_link2(qa_last_node, cur_node);
        qa_last_node = cur_node;
    }
    else
    {
        last_node->type = "GOAL";
    }

    topo_update();
    printf("[QA] qa stop\n");
}

void MainWindow::qa_loop()
{
    NODE* last_node = unimap.get_node_by_id(qa_last_node);
    if(last_node == NULL)
    {
        printf("[QA] last node empty\n");
        qa_timer.stop();
        return;
    }

    Eigen::Matrix4d cur_tf = slam.get_cur_tf();

    double d = (cur_tf.block(0,3,3,1) - last_node->tf.block(0,3,3,1)).norm();
    if(d > config.ANNOT_QA_STEP)
    {
        QString cur_node = unimap.add_node(cur_tf, "ROUTE");
        unimap.add_link2(qa_last_node, cur_node);
        qa_last_node = cur_node;

        topo_update();
    }
}

// for autocontrol
void MainWindow::bt_AutoMove()
{
    if(unimap.is_loaded == false)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    // get goal
    Eigen::Matrix4d goal_tf = Eigen::Matrix4d::Identity();
    if(pick.cur_node != "")
    {
        NODE* node = unimap.get_node_by_id(pick.cur_node);
        if(node != NULL)
        {
            goal_tf = node->tf;
            printf("[MAIN] automove, %s\n", pick.cur_node.toLocal8Bit().data());
        }
    }
    else
    {
        if(pick.last_btn == 1)
        {
            goal_tf = se2_to_TF(pick.r_pose);
            printf("[MAIN] automove, %.2f, %.2f, %.2f\n", pick.r_pose[0], pick.r_pose[1], pick.r_pose[2]*R2D);
        }
        else
        {
            printf("[MAIN] goal empty\n");
            return;
        }
    }

    // pure pursuit
    ctrl.move_pp(goal_tf, 0);

    // for path drawing
    is_path_update = true;
}

void MainWindow::bt_AutoMove2()
{
    if(unimap.is_loaded == false)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    // get goal
    Eigen::Matrix4d goal_tf = Eigen::Matrix4d::Identity();
    if(pick.cur_node != "")
    {
        NODE* node = unimap.get_node_by_id(pick.cur_node);
        if(node != NULL)
        {
            goal_tf = node->tf;
            printf("[MAIN] automove, %s\n", pick.cur_node.toLocal8Bit().data());
        }
    }
    else
    {
        if(pick.last_btn == 1)
        {
            goal_tf = se2_to_TF(pick.r_pose);
            printf("[MAIN] automove, %.2f, %.2f, %.2f\n", pick.r_pose[0], pick.r_pose[1], pick.r_pose[2]*R2D);
        }
        else
        {
            printf("[MAIN] goal empty\n");
            return;
        }
    }

    // holonomic pure pursuit
    ctrl.move_hpp(goal_tf, 0);

    // for path drawing
    is_path_update = true;
}

void MainWindow::bt_AutoMove3()
{
    if(unimap.is_loaded == false)
    {
        printf("[MAIN] check map load\n");
        return;
    }

    // get goal
    Eigen::Matrix4d goal_tf = Eigen::Matrix4d::Identity();
    if(pick.cur_node != "")
    {
        NODE* node = unimap.get_node_by_id(pick.cur_node);
        if(node != NULL)
        {
            goal_tf = node->tf;
            printf("[MAIN] automove, %s\n", pick.cur_node.toLocal8Bit().data());
        }
    }
    else
    {
        if(pick.last_btn == 1)
        {
            goal_tf = se2_to_TF(pick.r_pose);
            printf("[MAIN] automove, %.2f, %.2f, %.2f\n", pick.r_pose[0], pick.r_pose[1], pick.r_pose[2]*R2D);
        }
        else
        {
            printf("[MAIN] goal empty\n");
            return;
        }
    }

    // turn and go
    ctrl.move_tng(goal_tf, 0);

    // for path drawing
    is_path_update = true;
}

void MainWindow::bt_AutoStop()
{
    // stop driving
    ctrl.stop();

    // for path drawing
    is_path_update = true;
}

// for ws
void MainWindow::ws_command_motorinit(double time)
{
    mobile.motor_on();
}

void MainWindow::ws_command_move(double time, double vx, double vy, double wz)
{
    mobile.move(vx, vy, wz*D2R);
}

// for obsmap
void MainWindow::bt_ObsClear()
{
    obsmap.clear();
}

// watchdog
void MainWindow::watchdog_loop()
{
    watchdog_count++;

    // for 100ms loop
    if(ws.is_connected)
    {
        ws.send_status();
    }

    // for 1 sec loop
    if(watchdog_count % 10 == 0)
    {
        // check mobile
        if(mobile.is_connected)
        {
            if(mobile.is_synced == false)
            {
                mobile.sync();
                printf("[WATCH] try time sync, pc and mobile\n");
            }

            MOBILE_STATUS ms = mobile.get_status();
            if(ms.t != 0)
            {
                // when motor status 0, emo released, no charging
                if((ms.status_m0 == 0 || ms.status_m1 == 0) && ms.emo_state == 1 && ms.charge_state == 0)
                {
                    mobile.motor_on();
                }
            }
        }

        // check lidar
        if(lidar.is_connected_f)
        {
            if(lidar.is_synced_f == false)
            {
                lidar.sync_f();
                printf("[WATCH] try time sync, pc and front lidar\n");
            }
        }

        if(lidar.is_connected_b)
        {
            if(lidar.is_synced_b == false)
            {
                lidar.sync_b();
                printf("[WATCH] try time sync, pc and back lidar\n");
            }
        }

        // check camera


        // check loc
        if(slam.is_loc == false)
        {
            loc_fail_cnt = 0;
            slam.set_cur_loc_state("none");
        }
        else
        {
            Eigen::Vector2d ieir = slam.get_cur_ieir();
            if(ieir[0] > config.LOC_CHECK_IE || ieir[1] < config.LOC_CHECK_IR)
            {
                loc_fail_cnt++;
                if(loc_fail_cnt > 3)
                {
                    slam.set_cur_loc_state("fail");
                }
            }
            else
            {
                loc_fail_cnt = 0;
                slam.set_cur_loc_state("good");
            }
        }
    }
}

// plot
void MainWindow::plot_loop()
{
    // plot mobile info
    if(mobile.is_connected)
    {
        MOBILE_POSE mobile_pose = mobile.get_pose();
        QString mobile_pose_str;
        mobile_pose_str.sprintf("[MOBILE_POSE]\nt:%.3f, msg_q:%d\npos:%.2f,%.2f,%.2f\nvel:%.2f, %.2f, %.2f",
                                mobile_pose.t, (int)mobile.msg_que.unsafe_size(),
                                mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
                                mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D);
        ui->lb_MobilePoseInfo->setText(mobile_pose_str);

        // plot mobile status
        MOBILE_STATUS mobile_status = mobile.get_status();
        QString mobile_status_str;
        mobile_status_str.sprintf("[MOBILE_STATUS]\nconnection(m0,m1):%d,%d, status(m0,m1):%d,%d\ntemp(m0,m1): %d,%d, cur(m0,m1):%.2f,%.2f\ncharge,power,emo,remote:%d,%d,%d,%d\nBAT(in,out,cur):%.3f,%.3f,%.3f\npower:%.3f, total power:%.3f\ngyr:%.2f,%.2f,%.2f acc:%.3f,%.3f,%.3f",
                                  mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.status_m0, mobile_status.status_m1, mobile_status.temp_m0, mobile_status.temp_m1,
                                  (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0,
                                  mobile_status.charge_state, mobile_status.power_state, mobile_status.emo_state, mobile_status.remote_state,
                                  mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,
                                  mobile_status.power, mobile_status.total_power,
                                  mobile_status.imu_gyr_x, mobile_status.imu_gyr_y, mobile_status.imu_gyr_z,
                                  mobile_status.imu_acc_x, mobile_status.imu_acc_y, mobile_status.imu_acc_z);
        ui->lb_MobileStatusInfo->setText(mobile_status_str);

        // plot imu rpy
        Eigen::Vector3d mobile_rpy = mobile.get_imu();
        QString mobile_rpy_str;
        mobile_rpy_str.sprintf("[IMU_FILTERED]\nrx:%.3f, ry:%.3f, rz:%.3f\n", mobile_rpy[0]*R2D, mobile_rpy[1]*R2D, mobile_rpy[2]*R2D);
        ui->lb_ImuInfo->setText(mobile_rpy_str);
    }

    // plot slam info
    QString cur_loc_state = slam.get_cur_loc_state();
    Eigen::Vector2d cur_ieir = slam.get_cur_ieir();
    QString slam_info_str;
    slam_info_str.sprintf("[SLAM_INFO]\nmap_t: %.3f, %.3f\nloc_t: %.3f, %.3f\nkfrm_num: %d\nie: %.3f, ir: %.3f, loc_state: %s",
                          (double)slam.proc_time_map_a, (double)slam.proc_time_map_b,
                          (double)slam.proc_time_loc_a, (double)slam.proc_time_loc_b,
                          (int)slam.kfrm_storage.size(),
                          cur_ieir[0], cur_ieir[1], cur_loc_state.toLocal8Bit().data());
    ui->lb_SlamInfo->setText(slam_info_str);

    // plot que info
    QString que_info_str;
    que_info_str.sprintf("[QUES]\nscan_q:%d, msg_q:%d", (int)lidar.scan_que.unsafe_size(), (int)mobile.msg_que.unsafe_size());
    ui->lb_QueInfo->setText(que_info_str);

    // plot map & annotation
    if(unimap.is_loaded)
    {
        // draw map
        if(is_map_update)
        {
            is_map_update = false;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < unimap.kdtree_cloud.pts.size(); p++)
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

        // obsmap update
        if(is_obs_update)
        {
            is_obs_update = false;

            if(obsmap.octree != NULL)
            {
                cv::Mat obs_map;
                Eigen::Matrix4d obs_tf;
                obsmap.get_obs_map(obs_map, obs_tf);

                // plot grid map
                {
                    cv::Mat plot_obs_map;
                    cv::cvtColor(obs_map, plot_obs_map, cv::COLOR_GRAY2BGR);
                    obsmap.draw_robot(plot_obs_map, obs_tf);

                    ui->lb_Screen1->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot_obs_map)));
                    ui->lb_Screen1->setScaledContents(true);
                    ui->lb_Screen1->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
                }

                // obsmap boundary
                if(obsmap.mtx.try_lock())
                {
                    octomap::point3d bbx_min(obs_tf(0,3) - config.OBS_MAP_RANGE, obs_tf(1,3) - config.OBS_MAP_RANGE, -0.1);
                    octomap::point3d bbx_max(obs_tf(0,3) + config.OBS_MAP_RANGE, obs_tf(1,3) + config.OBS_MAP_RANGE, 0.1);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    for(octomap::OcTree::leaf_bbx_iterator it = obsmap.octree->begin_leafs_bbx(bbx_min, bbx_max, 16); it != obsmap.octree->end_leafs_bbx(); it++)
                    {
                        double x = it.getX();
                        double y = it.getY();
                        double z = it.getZ();
                        double prob = it->getOccupancy();

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

                    if(!viewer->updatePointCloud(cloud, "obs_pts"))
                    {
                        viewer->addPointCloud(cloud, "obs_pts");
                    }

                    // point size
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "obs_pts");

                    obsmap.mtx.unlock();
                }
            }
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

                    QString axis_id = id + "_axis";
                    if(viewer->contains(axis_id.toStdString()))
                    {
                        viewer->removeCoordinateSystem(axis_id.toStdString());
                    }

                    QString text_id = id + "_text";
                    if(viewer->contains(text_id.toStdString()))
                    {
                        viewer->removeShape(text_id.toStdString());
                    }
                }
                last_plot_nodes.clear();
            }

            // remove edges
            if(last_plot_edges.size() > 0)
            {
                for(size_t p = 0; p < last_plot_edges.size(); p++)
                {
                    QString id = last_plot_edges[p];
                    if(viewer->contains(id.toStdString()))
                    {
                        viewer->removeShape(id.toStdString());
                    }
                }
                last_plot_edges.clear();
            }

            // draw
            if(unimap.nodes.size() > 0)
            {
                if(ui->ckb_PlotNodes->isChecked())
                {
                    // draw nodes
                    for(size_t p = 0; p < unimap.nodes.size(); p++)
                    {
                        QString id = unimap.nodes[p].id;
                        if(unimap.nodes[p].type == "ROUTE")
                        {
                            viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.15, 1.0, 1.0, 1.0, id.toStdString());
                            viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());
                        }
                        else if(unimap.nodes[p].type == "GOAL")
                        {
                            viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                             config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                             0, 0.1, 0.5, 1.0, 0.0, id.toStdString());

                            QString axis_id = id + "_axis";
                            viewer->addCoordinateSystem(1.0, axis_id.toStdString());
                            viewer->updateCoordinateSystemPose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));

                            viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());
                        }
                        else if(unimap.nodes[p].type == "OBS")
                        {
                            viewer->addCube(-VIRTUAL_OBS_SIZE/2, VIRTUAL_OBS_SIZE/2,
                                            -VIRTUAL_OBS_SIZE/2, VIRTUAL_OBS_SIZE/2,
                                            0.0, VIRTUAL_OBS_SIZE, 0.0, 1.0, 1.0, id.toStdString());

                            viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());
                        }

                        // for erase
                        last_plot_nodes.push_back(id);
                    }
                }

                if(ui->ckb_PlotEdges->isChecked())
                {
                    // draw edges
                    for(size_t p = 0; p < unimap.nodes.size(); p++)
                    {
                        Eigen::Vector3d P0 = unimap.nodes[p].tf.block(0,3,3,1);
                        for(size_t q = 0; q < unimap.nodes[p].linked.size(); q++)
                        {
                            NODE* node = unimap.get_node_by_id(unimap.nodes[p].linked[q]);
                            Eigen::Vector3d P1 = node->tf.block(0,3,3,1);
                            Eigen::Vector3d P_mid = (P0+P1)/2;

                            pcl::PointXYZ pt0(P0[0], P0[1], P0[2]);
                            pcl::PointXYZ pt1(P_mid[0], P_mid[1], P_mid[2]);
                            pcl::PointXYZ pt2(P1[0], P1[1], P1[2]);

                            QString id0 = unimap.nodes[p].id + "_" + node->id + "_0";
                            QString id1 = unimap.nodes[p].id + "_" + node->id + "_1";

                            viewer->addLine(pt0, pt1, 1.0, 0.0, 0.0, id0.toStdString());
                            viewer->addLine(pt1, pt2, 0.0, 1.0, 0.0, id1.toStdString());

                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id0.toStdString());
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id1.toStdString());
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id0.toStdString());
                            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id1.toStdString());

                            last_plot_edges.push_back(id0);
                            last_plot_edges.push_back(id1);
                        }
                    }
                }

                if(ui->ckb_PlotNames->isChecked())
                {
                    // draw nodes
                    for(size_t p = 0; p < unimap.nodes.size(); p++)
                    {
                        QString id = unimap.nodes[p].id;
                        if(unimap.nodes[p].type == "GOAL")
                        {
                            // plot text
                            QString text_id = id + "_text";
                            pcl::PointXYZ position;
                            position.x = unimap.nodes[p].tf(0,3);
                            position.y = unimap.nodes[p].tf(1,3);
                            position.z = unimap.nodes[p].tf(2,3) + 1.0;
                            viewer->addText3D(id.toStdString(), position, 0.2, 0.0, 1.0, 0.0, text_id.toStdString());
                        }
                    }
                }
            }
        }

    }

    // plot cloud data
    if(slam.is_slam)
    {
        // mapping
        if(slam.mtx.try_lock())
        {
            // remove first
            if(viewer->contains("raw_pts"))
            {
                viewer->removePointCloud("raw_pts");
            }

            // plot live cloud
            if(ui->ckb_PlotLive->isChecked())
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(size_t p = 0; p < slam.live_cloud.pts.size(); p++)
                {
                    // set pos
                    pcl::PointXYZRGB pt;
                    pt.x = slam.live_cloud.pts[p].x;
                    pt.y = slam.live_cloud.pts[p].y;
                    pt.z = slam.live_cloud.pts[p].z;

                    // set color
                    double reflect = std::sqrt(slam.live_cloud.pts[p].r/255);
                    tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                    pt.r = c.r()*255;
                    pt.g = c.g()*255;
                    pt.b = c.b()*255;

                    // dynamic object
                    if(slam.live_cloud.pts[p].do_cnt < config.SLAM_ICP_DO_ACCUM_NUM)
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

            // plot ketframe pts            
            {
                int kfrm_id;
                if(slam.kfrm_update_que.try_pop(kfrm_id))
                {
                    QString name;
                    name.sprintf("kfrm_%d", kfrm_id);
                    if(!viewer->contains(name.toStdString()))
                    {
                        KFRAME kfrm = slam.kfrm_storage[kfrm_id];

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
                            viewer->updatePointCloudPose(name.toStdString(), Eigen::Affine3f(slam.kfrm_storage[kfrm_id].opt_G.cast<float>()));
                            last_plot_kfrms.push_back(name);
                        }

                        // point size
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, name.toStdString());
                    }
                    else
                    {
                        viewer->updatePointCloudPose(name.toStdString(), Eigen::Affine3f(slam.kfrm_storage[kfrm_id].opt_G.cast<float>()));

                        // point size
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, name.toStdString());
                    }
                }
            }

            slam.mtx.unlock();
        }
    }
    else if(slam.is_loc)
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

        // localzation
        TIME_POSE_PTS cur_tpp = slam.get_cur_tpp();

        // draw points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < cur_tpp.pts.size(); p++)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cur_tpp.pts[p][0];
            pt.y = cur_tpp.pts[p][1];
            pt.z = cur_tpp.pts[p][2];
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            cloud->push_back(pt);
        }

        if(!viewer->updatePointCloud(cloud, "raw_pts"))
        {
            viewer->addPointCloud(cloud, "raw_pts");
        }

        // pose update
        viewer->updatePointCloudPose("raw_pts",Eigen::Affine3f(cur_tpp.tf.cast<float>()));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, "raw_pts");        

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

            std::vector<Eigen::Matrix4d> traj = ctrl.calc_trajectory(Eigen::Vector3d(mobile.vx0, mobile.vy0, mobile.wz0), 0.2, 3.0, cur_tpp.tf);
            for(size_t p = 0; p < traj.size(); p++)
            {
                QString name;
                name.sprintf("traj_%d", p);

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
    }
    else
    {
        // get tf
        Eigen::Matrix4d cur_tf = slam.get_cur_tf();

        /*
        // raw data
        std::vector<Eigen::Vector3d> cur_scan = lidar.get_cur_scan();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < cur_scan.size(); p++)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cur_scan[p][0];
            pt.y = cur_scan[p][1];
            pt.z = cur_scan[p][2];
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;

            cloud->push_back(pt);
        }
        */

        // raw data
        std::vector<Eigen::Vector3d> cur_scan_f = lidar.get_cur_scan_f();
        std::vector<Eigen::Vector3d> cur_scan_b = lidar.get_cur_scan_b();

        // front lidar red
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < cur_scan_f.size(); p++)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cur_scan_f[p][0];
            pt.y = cur_scan_f[p][1];
            pt.z = cur_scan_f[p][2];
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;

            cloud->push_back(pt);
        }

        // rear lidar blue
        for(size_t p = 0; p < cur_scan_b.size(); p++)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cur_scan_b[p][0];
            pt.y = cur_scan_b[p][1];
            pt.z = cur_scan_b[p][2];
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;

            cloud->push_back(pt);
        }

        if(!viewer->updatePointCloud(cloud, "raw_pts"))
        {
            viewer->addPointCloud(cloud, "raw_pts");
        }

        // pose update
        viewer->updatePointCloudPose("raw_pts",Eigen::Affine3f(cur_tf.cast<float>()));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, "raw_pts");
    }

    // plot global path
    if(is_path_update)
    {
        // clear flag
        is_path_update = false;

        // erase first
        if(last_plot_global_path.size() > 0)
        {
            for(size_t p = 0; p < last_plot_global_path.size(); p++)
            {
                QString name = last_plot_global_path[p];
                if(viewer->contains(name.toStdString()))
                {
                    viewer->removeShape(name.toStdString());
                }
            }
            last_plot_global_path.clear();
        }

        // draw
        PATH global_path = ctrl.get_cur_global_path();
        if(global_path.pos.size() >= 2)
        {
            for(size_t p = 0; p < global_path.pos.size()-1; p++)
            {
                QString name;
                name.sprintf("global_path_%d_%d", (int)p, (int)p+1);

                Eigen::Vector3d P0 = global_path.pos[p];
                Eigen::Vector3d P1 = global_path.pos[p+1];

                pcl::PointXYZ pt0(P0[0], P0[1], P0[2] + 0.01);
                pcl::PointXYZ pt1(P1[0], P1[1], P1[2] + 0.01);

                viewer->addLine(pt0, pt1, 1.0, 0.0, 0.0, name.toStdString());
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, name.toStdString());
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name.toStdString());

                last_plot_global_path.push_back(name);
            }
        }
    }

    // plot local path
    {
        PATH local_path = ctrl.get_cur_local_path();

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
        if(local_path.pos.size() >= 2)
        {
            if(local_path.pos.size()/10 >= 2)
            {
                int last_p = 0;
                for(size_t p = 1; p < local_path.pos.size(); p++)
                {
                    if(p == local_path.pos.size()-1 || p % 10 == 0)
                    {
                        QString name;
                        name.sprintf("local_path_%d_%d", last_p, (int)p);

                        Eigen::Vector3d P0 = local_path.pos[last_p];
                        Eigen::Vector3d P1 = local_path.pos[p];

                        pcl::PointXYZ pt0(P0[0], P0[1], P0[2] + 0.02);
                        pcl::PointXYZ pt1(P1[0], P1[1], P1[2] + 0.02);

                        viewer->addLine(pt0, pt1, 1.0, 0.5, 0.0, name.toStdString());
                        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, name.toStdString());
                        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name.toStdString());

                        last_plot_local_path.push_back(name);
                        last_p = p;
                    }
                }
            }
            else
            {
                int last_p = 0;
                for(size_t p = 1; p < local_path.pos.size(); p++)
                {
                    QString name;
                    name.sprintf("local_path_%d_%d", last_p, (int)p);

                    Eigen::Vector3d P0 = local_path.pos[last_p];
                    Eigen::Vector3d P1 = local_path.pos[p];

                    pcl::PointXYZ pt0(P0[0], P0[1], P0[2] + 0.02);
                    pcl::PointXYZ pt1(P1[0], P1[1], P1[2] + 0.02);

                    viewer->addLine(pt0, pt1, 1.0, 0.5, 0.0, name.toStdString());
                    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, name.toStdString());
                    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name.toStdString());

                    last_plot_local_path.push_back(name);
                    last_p = p;
                }
            }
        }
    }


    // draw robot    
    {
        Eigen::Matrix4d cur_tf = slam.get_cur_tf();

        // draw axis
        if(viewer->contains("robot_axis"))
        {
            viewer->removeCoordinateSystem("robot_axis");
        }
        viewer->addCoordinateSystem(1.0, "robot_axis");
        viewer->updateCoordinateSystemPose("robot_axis", Eigen::Affine3f(cur_tf.cast<float>()));

        // draw body
        if(viewer->contains("robot_body"))
        {
            viewer->removeShape("robot_body");
        }
        viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                        config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.0, 0.5, 1.0, "robot_body");
        viewer->updateShapePose("robot_body", Eigen::Affine3f(cur_tf.cast<float>()));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "robot_body");
    }

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

        if(viewer->contains("sel_pre"))
        {
            viewer->removeShape("sel_pre");
        }

        if(pick.cur_node != "")
        {
            NODE *node = unimap.get_node_by_id(pick.cur_node);
            if(node != NULL)
            {
                pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 1.0, 1.0, 1.0);
                viewer->addPolygonMesh(donut, "sel_cur");
            }
        }

        if(pick.last_btn == 1)
        {
            // draw pose
            viewer->addCoordinateSystem(1.0, "O_pick");
            viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                            config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                            config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.5, 0.5, 0.5, "pick_body");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");

            viewer->updateShapePose("pick_body", Eigen::Affine3f(se2_to_TF(pick.r_pose).cast<float>()));
            viewer->updateCoordinateSystemPose("O_pick", Eigen::Affine3f(se2_to_TF(pick.r_pose).cast<float>()));
        }
    }

    // draw control points
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

        if(viewer->contains("replan_st"))
        {
            viewer->removeShape("replan_st");
        }

        // draw monitoring points
        {
            ctrl.mtx.lock();
            Eigen::Vector3d cur_pos = ctrl.last_cur_pos;
            Eigen::Vector3d tgt_pos = ctrl.last_tgt_pos;
            Eigen::Vector3d replan_st = ctrl.last_replan_st;
            Eigen::Vector3d local_goal = ctrl.last_local_goal;
            ctrl.mtx.unlock();

            viewer->addSphere(pcl::PointXYZ(cur_pos[0], cur_pos[1], cur_pos[2]), 0.1, 1.0, 1.0, 1.0, "cur_pos");
            viewer->addSphere(pcl::PointXYZ(tgt_pos[0], tgt_pos[1], tgt_pos[2]), 0.1, 1.0, 0.0, 0.0, "tgt_pos");
            viewer->addSphere(pcl::PointXYZ(replan_st[0], replan_st[1], replan_st[2]), 0.1, 0.0, 0.0, 1.0, "replan_st");
            viewer->addSphere(pcl::PointXYZ(local_goal[0], local_goal[1], local_goal[2]), 0.1, 0.0, 1.0, 0.0, "local_goal");

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cur_pos");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "tgt_pos");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "replan_st");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "local_goal");
        }
    }

    // cam control
    if(ui->cb_ViewType->currentText() == "FREE")
    {

    }
    else if(ui->cb_ViewType->currentText() == "FOLLOW")
    {

    }

    // rendering
    ui->qvtkWidget->renderWindow()->Render();
}

void MainWindow::plot_loop2()
{
    // plot map & annotation
    if(unimap.is_loaded)
    {
        // draw map
        if(is_map_update2)
        {
            is_map_update2 = false;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(size_t p = 0; p < unimap.kdtree_cloud.pts.size(); p++)
            {
                pcl::PointXYZRGB pt;
                pt.x = unimap.kdtree_cloud.pts[p].x;
                pt.y = unimap.kdtree_cloud.pts[p].y;
                pt.z = unimap.kdtree_cloud.pts[p].z;

                double reflect = std::sqrt(unimap.kdtree_cloud.pts[p].r/255);
                tinycolormap::Color c = tinycolormap::GetColor(reflect, tinycolormap::ColormapType::Viridis);

                pt.r = c.r()*255;
                pt.g = c.g()*255;
                pt.b = c.b()*255;

                if(unimap.kdtree_mask[p] == 0)
                {
                    pt.r = 255;
                    pt.g = 0;
                    pt.b = 0;
                }

                cloud->push_back(pt);
            }

            if(!viewer2->updatePointCloud(cloud, "map_pts"))
            {
                viewer2->addPointCloud(cloud, "map_pts");
            }

            // point size
            viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, "map_pts");
        }

        // draw topo
        if(is_topo_update2)
        {
            is_topo_update2 = false;

            // erase first
            if(last_plot_nodes2.size() > 0)
            {
                for(size_t p = 0; p < last_plot_nodes2.size(); p++)
                {
                    QString id = last_plot_nodes2[p];
                    if(viewer2->contains(id.toStdString()))
                    {
                        viewer2->removeShape(id.toStdString());
                    }

                    QString axis_id = id + "_axis";
                    if(viewer2->contains(axis_id.toStdString()))
                    {
                        viewer2->removeCoordinateSystem(axis_id.toStdString());
                    }

                    QString text_id = id + "_text";
                    if(viewer2->contains(text_id.toStdString()))
                    {
                        viewer2->removeShape(text_id.toStdString());
                    }
                }
                last_plot_nodes2.clear();
            }

            // remove edges
            if(last_plot_edges2.size() > 0)
            {
                for(size_t p = 0; p < last_plot_edges2.size(); p++)
                {
                    QString id = last_plot_edges2[p];
                    if(viewer2->contains(id.toStdString()))
                    {
                        viewer2->removeShape(id.toStdString());
                    }
                }
                last_plot_edges2.clear();
            }

            // draw
            if(unimap.nodes.size() > 0)
            {
                if(ui->ckb_PlotNodes2->isChecked())
                {
                    // draw nodes
                    for(size_t p = 0; p < unimap.nodes.size(); p++)
                    {
                        QString id = unimap.nodes[p].id;
                        if(unimap.nodes[p].type == "ROUTE")
                        {
                            viewer2->addSphere(pcl::PointXYZ(0, 0, 0), 0.15, 1.0, 1.0, 1.0, id.toStdString());
                            viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());
                        }
                        else if(unimap.nodes[p].type == "GOAL")
                        {
                            viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                             config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                             0, 0.1, 0.5, 1.0, 0.0, id.toStdString());


                            QString axis_id = id + "_axis";
                            viewer2->addCoordinateSystem(1.0, axis_id.toStdString());
                            viewer2->updateCoordinateSystemPose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));

                            viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());
                        }
                        else if(unimap.nodes[p].type == "OBS")
                        {
                            viewer2->addCube(-VIRTUAL_OBS_SIZE/2, VIRTUAL_OBS_SIZE/2,
                                            -VIRTUAL_OBS_SIZE/2, VIRTUAL_OBS_SIZE/2,
                                            0.0, VIRTUAL_OBS_SIZE, 0.0, 1.0, 1.0, id.toStdString());

                            viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());
                        }

                        // for erase
                        last_plot_nodes2.push_back(id);
                    }
                }

                if(ui->ckb_PlotEdges2->isChecked())
                {
                    // draw edges
                    for(size_t p = 0; p < unimap.nodes.size(); p++)
                    {
                        Eigen::Vector3d P0 = unimap.nodes[p].tf.block(0,3,3,1);
                        for(size_t q = 0; q < unimap.nodes[p].linked.size(); q++)
                        {
                            NODE* node = unimap.get_node_by_id(unimap.nodes[p].linked[q]);
                            Eigen::Vector3d P1 = node->tf.block(0,3,3,1);
                            Eigen::Vector3d P_mid = (P0+P1)/2;

                            pcl::PointXYZ pt0(P0[0], P0[1], P0[2]);
                            pcl::PointXYZ pt1(P_mid[0], P_mid[1], P_mid[2]);
                            pcl::PointXYZ pt2(P1[0], P1[1], P1[2]);

                            QString id0 = unimap.nodes[p].id + "_" + node->id + "_0";
                            QString id1 = unimap.nodes[p].id + "_" + node->id + "_1";

                            viewer2->addLine(pt0, pt1, 1.0, 0.0, 0.0, id0.toStdString());
                            viewer2->addLine(pt1, pt2, 0.0, 1.0, 0.0, id1.toStdString());

                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id0.toStdString());
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id1.toStdString());
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id0.toStdString());
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id1.toStdString());

                            last_plot_edges2.push_back(id0);
                            last_plot_edges2.push_back(id1);
                        }
                    }
                }

                if(ui->ckb_PlotNames2->isChecked())
                {
                    // draw nodes
                    for(size_t p = 0; p < unimap.nodes.size(); p++)
                    {
                        QString id = unimap.nodes[p].id;
                        if(unimap.nodes[p].type == "GOAL")
                        {
                            // plot text
                            QString text_id = id + "_text";
                            pcl::PointXYZ position;
                            position.x = unimap.nodes[p].tf(0,3);
                            position.y = unimap.nodes[p].tf(1,3);
                            position.z = unimap.nodes[p].tf(2,3) + 1.0;
                            viewer2->addText3D(id.toStdString(), position, 0.2, 0.0, 1.0, 0.0, text_id.toStdString());
                        }
                    }
                }
            }
        }
    }

    // draw robot    
    {
        Eigen::Matrix4d cur_tf = slam.get_cur_tf();

        // draw axis
        if(viewer2->contains("robot_axis"))
        {
            viewer2->removeCoordinateSystem("robot_axis");
        }
        viewer2->addCoordinateSystem(1.0, "robot_axis");
        viewer2->updateCoordinateSystemPose("robot_axis", Eigen::Affine3f(cur_tf.cast<float>()));

        // draw body
        if(viewer2->contains("robot_body"))
        {
            viewer2->removeShape("robot_body");
        }
        viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                        config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.0, 0.5, 1.0, "robot_body");
        viewer2->updateShapePose("robot_body", Eigen::Affine3f(cur_tf.cast<float>()));
        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "robot_body");
    }

    // draw cursors
    if(is_pick_update2)
    {
        is_pick_update2 = false;

        // plot node picking text
        if(unimap.is_loaded)
        {
            if(pick.pre_node != "")
            {
                NODE* pre_node = unimap.get_node_by_id(pick.pre_node);
                if(pre_node != NULL)
                {
                    Eigen::Vector3d pose = TF_to_se2(pre_node->tf);

                    QString str;
                    str.sprintf("[pre_node]\nid: %s\ntype: %s\npos: %.3f, %.3f, %.2f\ninfo: %s",
                                pre_node->id.toLocal8Bit().data(), pre_node->type.toLocal8Bit().data(),
                                pose[0], pose[1], pose[2]*R2D, pre_node->info.toLocal8Bit().data());

                    ui->lb_PreNodeInfo->setText(str);
                }
                else
                {
                    ui->lb_PreNodeInfo->setText("[pre_node]");
                }
            }
            else
            {
                ui->lb_PreNodeInfo->setText("[pre_node]");
            }

            if(pick.cur_node != "")
            {
                NODE* cur_node = unimap.get_node_by_id(pick.cur_node);
                if(cur_node != NULL)
                {
                    Eigen::Vector3d pose = TF_to_se2(cur_node->tf);

                    QString str;
                    str.sprintf("[cur_node]\nid: %s\ntype: %s\npos: %.3f, %.3f, %.2f\ninfo: %s",
                                cur_node->id.toLocal8Bit().data(), cur_node->type.toLocal8Bit().data(),
                                pose[0], pose[1], pose[2]*R2D, cur_node->info.toLocal8Bit().data());

                    ui->lb_CurNodeInfo->setText(str);
                }
                else
                {
                    ui->lb_CurNodeInfo->setText("[cur_node]");
                }
            }
            else
            {
                ui->lb_CurNodeInfo->setText("[cur_node]");
            }
        }

        // erase first
        if(viewer2->contains("O_pick"))
        {
            viewer2->removeCoordinateSystem("O_pick");
        }

        if(viewer2->contains("pick_body"))
        {
            viewer2->removeShape("pick_body");
        }

        if(viewer2->contains("sel_cur"))
        {
            viewer2->removeShape("sel_cur");
        }

        if(viewer2->contains("sel_pre"))
        {
            viewer2->removeShape("sel_pre");
        }

        // different behavior each tab
        if(ui->annot_tab->tabText(ui->annot_tab->currentIndex()) == "EDIT_MAP")
        {
            if(pick.l_drag)
            {
                Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(pick.l_pt0[0], pick.l_pt0[1], 0));

                pcl::PolygonMesh donut = make_donut(0.15, 0.01, tf, 1.0, 0.0, 0.0, 0.5);
                viewer2->addPolygonMesh(donut, "pick_body");

                // erase
                unimap.set_cloud_mask(pick.l_pt0, 0.15, 0);
                map_update();
            }
            else if(pick.r_drag)
            {
                Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(pick.r_pt0[0], pick.r_pt0[1], 0));

                pcl::PolygonMesh donut = make_donut(0.15, 0.01, tf, 0.0, 1.0, 0.0, 0.5);
                viewer2->addPolygonMesh(donut, "pick_body");

                // restore
                unimap.set_cloud_mask(pick.r_pt0, 0.15, 1);
                map_update();
            }
        }
        else if(ui->annot_tab->tabText(ui->annot_tab->currentIndex()) == "EDIT_TOPO")
        {
            // draw selection
            if(pick.pre_node != "")
            {
                NODE *node = unimap.get_node_by_id(pick.pre_node);
                if(node != NULL)
                {
                    pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 1.0, 0.0, 0.0);
                    viewer2->addPolygonMesh(donut, "sel_pre");
                }
            }

            if(pick.cur_node != "")
            {
                NODE *node = unimap.get_node_by_id(pick.cur_node);
                if(node != NULL)
                {
                    pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 0.0, 1.0, 0.0);
                    viewer2->addPolygonMesh(donut, "sel_cur");
                }
            }

            if(pick.last_btn == 1)
            {
                // draw pose
                viewer2->addCoordinateSystem(1.0, "O_pick");
                if(ui->cb_NodeType->currentText() == "ROUTE")
                {
                    viewer2->addSphere(pcl::PointXYZ(0, 0, 0), 0.15, 0.5, 0.5, 0.5, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(se2_to_TF(pick.r_pose).cast<float>()));
                }
                else if(ui->cb_NodeType->currentText() == "GOAL")
                {
                    viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                     config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                     config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.5, 0.5, 0.5, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(se2_to_TF(pick.r_pose).cast<float>()));
                }
                else if(ui->cb_NodeType->currentText() == "OBS")
                {
                    viewer2->addCube(-VIRTUAL_OBS_SIZE/2, VIRTUAL_OBS_SIZE/2,
                                     -VIRTUAL_OBS_SIZE/2, VIRTUAL_OBS_SIZE/2,
                                     0.0, VIRTUAL_OBS_SIZE, 0.0, 1.0, 1.0, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(se2_to_TF(pick.r_pose).cast<float>()));
                }

                viewer2->updateCoordinateSystemPose("O_pick", Eigen::Affine3f(se2_to_TF(pick.r_pose).cast<float>()));
            }
        }
    }

    // rendering
    ui->qvtkWidget2->renderWindow()->Render();
}
