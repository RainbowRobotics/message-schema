#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , config(this)
    , logger(this)
    , mobile(this)
    , lidar(this)
    , blidar(this)
    , cam(this)
    , code(this)
    , unimap(this)
    , obsmap(this)
    , slam(this)
    , ctrl(this)
    , dctrl(this)
    , task(this)
    , sim(this)
    , cfms(this)
    , cms(this)
    , cui(this)    
    , system_logger(this)
    , ui(new Ui::MainWindow)
    , plot_timer(this)
    , plot_timer2(this)
    , qa_timer(this)
{
    ui->setupUi(this);

    // for tab
    connect(ui->main_tab, SIGNAL(currentChanged(int)), this, SLOT(all_update()));
    connect(ui->annot_tab, SIGNAL(currentChanged(int)), this, SLOT(all_update()));

    // for 3d viewer
    connect(ui->bt_ViewReset, SIGNAL(clicked()), this, SLOT(bt_ViewReset()));
    connect(ui->bt_SetTopView, SIGNAL(clicked()), this, SLOT(bt_SetTopView()));
    connect(ui->ckb_PlotLive, SIGNAL(stateChanged(int)), this, SLOT(map_update()));
    connect(ui->ckb_PlotNodes, SIGNAL(stateChanged(int)), this, SLOT(topo_update()));
    connect(ui->ckb_PlotEdges, SIGNAL(stateChanged(int)), this, SLOT(topo_update()));
    connect(ui->ckb_PlotNames, SIGNAL(stateChanged(int)), this, SLOT(topo_update()));
    connect(ui->ckb_PlotNodes2, SIGNAL(stateChanged(int)), this, SLOT(topo_update()));
    connect(ui->ckb_PlotEdges2, SIGNAL(stateChanged(int)), this, SLOT(topo_update()));
    connect(ui->ckb_PlotNames2, SIGNAL(stateChanged(int)), this, SLOT(topo_update()));

    // timer
    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    connect(&plot_timer2, SIGNAL(timeout()), this, SLOT(plot_loop2()));
    connect(&qa_timer, SIGNAL(timeout()), this, SLOT(qa_loop()));

    // config
    connect(ui->bt_ConfigLoad, SIGNAL(clicked()), this, SLOT(bt_ConfigLoad()));

    // emergency stop
    connect(ui->bt_Emergency, SIGNAL(clicked()), this, SLOT(bt_Emergency()));

    // motor
    connect(ui->bt_MotorInit, SIGNAL(clicked()), this, SLOT(bt_MotorInit()));

    // test
    connect(ui->bt_Sync, SIGNAL(clicked()), this, SLOT(bt_Sync()));
    connect(ui->bt_MoveLinear, SIGNAL(clicked()), this, SLOT(bt_MoveLinear()));
    connect(ui->bt_MoveRotate, SIGNAL(clicked()), this, SLOT(bt_MoveRotate()));
    connect(ui->bt_Test, SIGNAL(clicked()), this, SLOT(bt_Test()));
    connect(ui->bt_TestLed, SIGNAL(clicked()), this, SLOT(bt_TestLed()));
    connect(ui->ckb_TestDebug, SIGNAL(stateChanged(int)), this, SLOT(ckb_TestDebug()));
    connect(ui->bt_TestImgSave, SIGNAL(clicked()), this, SLOT(bt_TestImgSave()));

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
    connect(ui->bt_LocInitSemiAuto, SIGNAL(clicked()), this, SLOT(bt_LocInitSemiAuto()));
    connect(ui->bt_LocInitAuto, SIGNAL(clicked()), this, SLOT(bt_LocInitAuto()));
    connect(ui->bt_LocStart, SIGNAL(clicked()), this, SLOT(bt_LocStart()));
    connect(ui->bt_LocStop, SIGNAL(clicked()), this, SLOT(bt_LocStop()));

    // annotation
    connect(ui->bt_MapLoad2, SIGNAL(clicked()), this, SLOT(bt_MapLoad()));
    connect(ui->bt_MapReload, SIGNAL(clicked()), this, SLOT(bt_MapReload()));
    connect(ui->bt_MapSave2, SIGNAL(clicked()), this, SLOT(bt_MapSave2()));
    connect(ui->bt_AddNode, SIGNAL(clicked()), this, SLOT(bt_AddNode()));
    connect(ui->bt_AddLink1, SIGNAL(clicked()), this, SLOT(bt_AddLink1()));
    connect(ui->bt_AddLink2, SIGNAL(clicked()), this, SLOT(bt_AddLink2()));
    connect(ui->bt_AutoLink, SIGNAL(clicked()), this, SLOT(bt_AutoLink()));
    connect(ui->bt_EditNodePos, SIGNAL(clicked()), this, SLOT(bt_EditNodePos()));
    connect(ui->bt_EditNodeType, SIGNAL(clicked()), this, SLOT(bt_EditNodeType()));
    connect(ui->bt_EditNodeInfo, SIGNAL(clicked()), this, SLOT(bt_EditNodeInfo()));    
    connect(ui->bt_EditNodeName, SIGNAL(clicked()), this, SLOT(bt_EditNodeName()));
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
    connect(ui->bt_QuickAddNode, SIGNAL(clicked()), this, SLOT(bt_QuickAddNode()));

    connect(ui->spb_NodeSizeX, SIGNAL(valueChanged(double)), this, SLOT(pick_update()));
    connect(ui->spb_NodeSizeY, SIGNAL(valueChanged(double)), this, SLOT(pick_update()));
    connect(ui->spb_NodeSizeZ, SIGNAL(valueChanged(double)), this, SLOT(pick_update()));

    // for simulation
    connect(ui->bt_SimInit, SIGNAL(clicked()), this, SLOT(bt_SimInit()));

    // for autocontrol
    connect(ui->bt_AutoMove, SIGNAL(clicked()), this, SLOT(bt_AutoMove()));
    connect(ui->bt_AutoMove2, SIGNAL(clicked()), this, SLOT(bt_AutoMove2()));
    connect(ui->bt_AutoMove3, SIGNAL(clicked()), this, SLOT(bt_AutoMove3()));
    connect(ui->bt_AutoStop, SIGNAL(clicked()), this, SLOT(bt_AutoStop()));
    connect(ui->bt_AutoPause, SIGNAL(clicked()), this, SLOT(bt_AutoPause()));
    connect(ui->bt_AutoResume, SIGNAL(clicked()), this, SLOT(bt_AutoResume()));
    connect(&ctrl, SIGNAL(signal_local_path_updated()), this, SLOT(slot_local_path_updated()));
    connect(&ctrl, SIGNAL(signal_global_path_updated()), this, SLOT(slot_global_path_updated()));
    connect(&ctrl, SIGNAL(signal_move_succeed(QString)), &cms, SLOT(slot_move_succeed(QString)));
    connect(&ctrl, SIGNAL(signal_move_failed(QString)), &cms, SLOT(slot_move_failed(QString)));

    // for slam
    connect(&slam, SIGNAL(signal_localization_semiautoinit_succeed(QString)), &cms, SLOT(slot_localization_semiautoinit_succeed(QString)));
    connect(&slam, SIGNAL(signal_localization_semiautoinit_failed(QString)), &cms, SLOT(slot_localization_semiautoinit_failed(QString)));
    connect(&slam, SIGNAL(signal_localization_semiautoinit_succeed(QString)), &cui, SLOT(slot_localization_semiautoinit_succeed(QString)));
    connect(&slam, SIGNAL(signal_localization_semiautoinit_failed(QString)), &cui, SLOT(slot_localization_semiautoinit_failed(QString)));

    // for obsmap
    connect(&obsmap, SIGNAL(obs_updated()), this, SLOT(obs_update()));
    connect(ui->bt_ObsClear, SIGNAL(clicked()), this, SLOT(bt_ObsClear()));

    // task interface
    connect(ui->bt_TaskAdd, SIGNAL(clicked()), this, SLOT(bt_TaskAdd()));
    connect(ui->bt_TaskDel, SIGNAL(clicked()), this, SLOT(bt_TaskDel()));
    connect(ui->bt_TaskSave, SIGNAL(clicked()), this, SLOT(bt_TaskSave()));
    connect(ui->bt_TaskLoad, SIGNAL(clicked()), this, SLOT(bt_TaskLoad()));
    connect(ui->bt_TaskPlay, SIGNAL(clicked()), this, SLOT(bt_TaskPlay()));
    connect(ui->bt_TaskPause, SIGNAL(clicked()), this, SLOT(bt_TaskPause()));    
    connect(ui->bt_TaskCancel, SIGNAL(clicked()), this, SLOT(bt_TaskCancel()));

    // for websocket ui
    connect(this, SIGNAL(signal_send_status()), &cui, SLOT(send_status()));

    // for fms
    connect(this, SIGNAL(signal_send_info()), &cfms, SLOT(slot_send_info()));
    connect(ui->bt_SendMap, SIGNAL(clicked()), this, SLOT(bt_SendMap()));

    // for log
    connect(&logger, SIGNAL(signal_write_log(QString, QString)), this, SLOT(slot_write_log(QString, QString)));

    // for tab
    connect(ui->main_tab, &QTabWidget::currentChanged, this, &MainWindow::slot_main_tab_changed);
    connect(ui->menu_tab, &QTabWidget::currentChanged, this, &MainWindow::slot_menu_tab_changed);


    // solve tab with vtk render window problem
    QTimer::singleShot(100, [&]()
    {
        ui->main_tab->setCurrentIndex(1);
        QTimer::singleShot(100, [&]()
        {
            ui->main_tab->setCurrentIndex(0);

            QTimer::singleShot(2000, [&]()
            {
                // set plot window
                setup_vtk();

                // init modules
                init_modules();

                // start plot loop
                if(config.SIM_MODE == 1)
                {
                    plot_timer.start(50);
                    plot_timer2.start(50);
                }
                else
                {
                    plot_timer.start(100);
                    plot_timer2.start(100);
                }
            });
        });
    });
}

MainWindow::~MainWindow()
{
    if(watch_thread != NULL)
    {
        watch_flag = false;
        watch_thread->join();
        watch_thread = NULL;
    }

    if(comm_thread != NULL)
    {
        comm_flag = false;
        comm_thread->join();
        comm_thread = NULL;
    }

    if(jog_thread != NULL)
    {
        jog_flag = false;
        jog_thread->join();
        jog_thread = NULL;
    }

    delete ui;
}

// for emergency stop
void MainWindow::bt_Emergency()
{
    task.cancel();
    ctrl.stop();
    dctrl.stop();
    mobile.move(0,0,0);
}

// for tab
void MainWindow::slot_main_tab_changed()
{
    QWidget *cur_tab = ui->main_tab->currentWidget();
    if(cur_tab ==  ui->page_slamnav)
    {
        ui->menu_tab->setCurrentWidget(ui->tab_S);
    }
    else if(cur_tab == ui->page_annot)
    {
        ui->menu_tab->setCurrentWidget(ui->tab_A);
    }
}
void MainWindow::slot_menu_tab_changed()
{
    QWidget *cur_tab = ui->menu_tab->currentWidget();
    if(cur_tab ==  ui->tab_S)
    {
        ui->main_tab->setCurrentWidget(ui->page_slamnav);
    }
    else if(cur_tab == ui->tab_A)
    {
        ui->main_tab->setCurrentWidget(ui->page_annot);
    }
}

// for replot
void MainWindow::bt_ViewReset()
{
    is_view_reset = true;
}

void MainWindow::bt_SetTopView()
{
    is_set_top_view = true;
}

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
void MainWindow::init_modules()
{            
    // config module init
    #ifdef USE_SRV
    config.config_path = QCoreApplication::applicationDirPath() + "/config/SRV/config.json";
    config.config_sn_path = QCoreApplication::applicationDirPath() + "/config/SRV/config_sn.json";
    #endif

    #ifdef USE_AMR_400
    config.config_path = QCoreApplication::applicationDirPath() + "/config/AMR_400/config.json";
    config.config_sn_path = QCoreApplication::applicationDirPath() + "/config/AMR_400/config_sn.json";
    #endif

    #ifdef USE_AMR_400_LAKI
    config.config_path = QCoreApplication::applicationDirPath() + "/config/AMR_400_LAKI/config.json";
    config.config_sn_path = QCoreApplication::applicationDirPath() + "/config/AMR_400_LAKI/config_sn.json";
    #endif

    config.load();

    // simulation check
    if(config.SIM_MODE == 1)
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

    // log module init (system)
    system_logger.log_path = QCoreApplication::applicationDirPath() + "/systemlog/";
    system_logger.init();

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

    // bottom lidar module init
    blidar.config = &config;
    blidar.logger = &logger;
    blidar.mobile = &mobile;
    if(config.USE_BLIDAR)
    {
        blidar.open();
    }

    // code reader module init
    code.config = &config;
    code.logger = &logger;
    code.unimap = &unimap;
    if(config.USE_BQR)
    {
        code.open();
    }

    // cam module init
    cam.config = &config;
    cam.logger = &logger;
    cam.mobile = &mobile;
    if(config.USE_CAM)
    {
        cam.open();
    }

    // slam module init
    slam.config = &config;
    slam.logger = &logger;
    slam.mobile = &mobile;
    slam.lidar = &lidar;
    slam.blidar = &blidar;
    slam.cam = &cam;
    slam.unimap = &unimap;
    slam.obsmap = &obsmap;

    aruco.config = &config;
    aruco.logger = &logger;
    aruco.cam = &cam;
    aruco.slam = &slam;
    aruco.unimap = &unimap;
    if(config.USE_ARUCO)
    {
        aruco.init();
    }

    // docking control module init
    dctrl.config = &config;
    dctrl.logger = &logger;
    dctrl.mobile = &mobile;
    dctrl.lidar = &lidar;
    dctrl.cam = &cam;
    dctrl.code = &code;
    dctrl.slam = &slam;
    dctrl.unimap = &unimap;
    dctrl.obsmap = &obsmap;
    dctrl.init();

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

    // simulation module init
    sim.config = &config;
    sim.logger = &logger;
    sim.mobile = &mobile;
    sim.lidar = &lidar;
    sim.slam = &slam;
    sim.unimap = &unimap;

    // task init
    task.config = &config;
    task.unimap = &unimap;
    task.slam = &slam;
    task.obsmap = &obsmap;
    task.ctrl = &ctrl;
    task.mobile = &mobile;

    // fms client module init
    cfms.config = &config;
    cfms.logger = &logger;
    cfms.mobile = &mobile;
    cfms.slam = &slam;
    cfms.unimap = &unimap;
    cfms.obsmap = &obsmap;
    cfms.ctrl = &ctrl;
    cfms.init();

    // socket.io client init
    cms.config = &config;
    cms.logger = &logger;
    cms.mobile = &mobile;
    cms.lidar = &lidar;
    cms.cam = &cam;
    cms.code = &code;
    cms.slam = &slam;
    cms.unimap = &unimap;
    cms.obsmap = &obsmap;
    cms.ctrl = &ctrl;
    cms.init();

    // websocket ui init
    cui.config = &config;
    cui.logger = &logger;
    cui.mobile = &mobile;
    cui.lidar = &lidar;
    cui.cam = &cam;
    cui.code = &code;
    cui.slam = &slam;
    cui.unimap = &unimap;
    cui.obsmap = &obsmap;
    cui.ctrl = &ctrl;
    cui.init();

    /*
    // map auto load
    if(config.MAP_PATH != "")
    {
        QDir dir(config.MAP_PATH);
        if(dir.exists())
        {
            map_dir = config.MAP_PATH;
            unimap.load_map(map_dir);
            all_update();
        }
    }
    */

    // start jog loop
    jog_flag = true;
    jog_thread = new std::thread(&MainWindow::jog_loop, this);

    // start watchdog loop
    watch_flag = true;
    watch_thread = new std::thread(&MainWindow::watch_loop, this);

    // start watchdog loop
    comm_flag = true;
    comm_thread = new std::thread(&MainWindow::comm_loop, this);
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

void MainWindow::all_plot_clear()
{
    viewer->removeAllCoordinateSystems();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer2->removeAllCoordinateSystems();
    viewer2->removeAllPointClouds();
    viewer2->removeAllShapes();

    // add global axis
    viewer->addCoordinateSystem(1.0, "O_global");
    viewer2->addCoordinateSystem(1.0, "O_global");
}

// for picking interface
bool MainWindow::eventFilter(QObject *object, QEvent *ev)
{
    if(object == ui->qvtkWidget)
    {
        // cam control
        if(ui->cb_ViewType->currentText() == "VIEW_3D")
        {
            return false;
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
    else if(object == ui->qvtkWidget2)
    {
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
            // keyboard event
            if(ev->type() == QEvent::KeyPress)
            {
                QKeyEvent* ke = static_cast<QKeyEvent*>(ev);
                switch(ke->key())
                {
                    case Qt::Key_A:
                        is_grab = true;
                        pick.pre_node = "";
                        pick.cur_node = "";
                        break;
                    case Qt::Key_Q:
                        is_free_move = true;
                        break;
                    case Qt::Key_Control:
                        is_pressed_btn_ctrl = true;
                        break;
                    case Qt::Key_C:
                        is_pressed_btn_c = true;
                        break;
                    case Qt::Key_V:
                        is_pressed_btn_v = true;
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
                    case Qt::Key_A:
                        is_grab = false;
                        break;
                    case Qt::Key_Q:
                        is_free_move = false;
                        break;
                    case Qt::Key_Control:
                        is_pressed_btn_ctrl = false;
                        break;
                    case Qt::Key_C:
                        is_pressed_btn_c = false;
                        break;
                    case Qt::Key_V:
                        is_pressed_btn_v = false;
                        break;
                    case::Qt::Key_Escape:
                        selected_nodes.clear();
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
                    pick.l_drag = true;

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

                    if(is_grab == true)
                    {
                        // nodes clicked
                        lt_x = pt[0];
                        lt_y = pt[1];
                        rb_x = pt[0];
                        rb_y = pt[1];
                        contains_nodes.clear();
                    }
                    else if(is_grab == false)
                    {
                        // only one node clicked
                        std::vector<QString> _selected_nodes;
                        _selected_nodes.push_back(pick.cur_node);
                        selected_nodes = _selected_nodes;

                        lt_x = 0.0;
                        lt_y = 0.0;
                        rb_x = 0.0;
                        rb_y = 0.0;
                        copy_contains_nodes.clear();
                        contains_nodes.clear();
                    }

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
                    // ray casting
                    double x = me->pos().x();
                    double y = me->pos().y();
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction, viewer2);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.l_pt1 = pt;

                    if(is_grab == true)
                    {
                        rb_x = pt[0];
                        rb_y = pt[1];
                    }

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
                    pick.l_drag = false;
                    if(is_grab == true)
                    {
                        if(unimap.is_loaded)
                        {
                            std::vector<QString> nodes = unimap.get_nodes();
                            for(size_t p=0; p < nodes.size(); p++)
                            {
                                NODE* node = unimap.get_node_by_id(nodes[p]);
                                if(node == NULL)
                                {
                                    continue;
                                }

                                double x = node->tf(0,3);
                                double y = node->tf(1,3);

                                if(x >= lt_x && x <= rb_x && y >= rb_y && y <= lt_y)
                                {
                                    contains_nodes.push_back(node->id);
                                }
                            }

                            selected_nodes = contains_nodes;

                            copy_contains_nodes.clear();
                            pick.pre_node = "";
                            pick.cur_node = "";
                        }
                    }

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
    if(unimap.is_loaded == false)
    {
        logger.write_log("[SIM] map load first", "Red", true, false);
        return;
    }

    // stop first
    if(slam.is_loc)
    {
        slam.localization_stop();
    }
    slam.cur_tf = Eigen::Matrix4d::Identity();
    sim.stop();

    // start
    sim.cur_tf = se2_to_TF(pick.r_pose);
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

// for mapping and localization
void MainWindow::bt_MapBuild()
{
    if(config.SIM_MODE == 1)
    {
        logger.write_log("[MAIN] map build not allowed SIM_MODE", "Red", true, false);
        return;
    }

    // stop first
    slam.localization_stop();
    slam.mapping_stop();

    // clear first
    unimap.clear();
    all_plot_clear();

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
        logger.write_log("[MAIN] map save not allowed SIM_MODE", "Red", true, false);
        return;
    }

    // check
    if(map_dir == "")
    {
        logger.write_log("[MAIN] no map_dir", "Red", true, false);
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

    config.MAP_PATH = map_dir;
    config.save_map_path();
}

void MainWindow::bt_MapLoad()
{
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    if(!path.isNull())
    {
        slam.localization_stop();
        obsmap.clear();

        map_dir = path;
        unimap.load_map(path);
        all_update();

        config.MAP_PATH = map_dir;
        config.save_map_path();
    }
}

void MainWindow::bt_LocInit()
{
    // manual init
    slam.mtx.lock();
    slam.cur_tf = se2_to_TF(pick.r_pose);
    slam.mtx.unlock();

    if(config.SIM_MODE)
    {
        bt_SimInit();
    }
}

void MainWindow::bt_LocInitSemiAuto()
{
    if(slam.is_busy)
    {
        printf("[AUTO_INIT] semi-auto init already running\n");
        return;
    }

    logger.write_log("[AUTO_INIT] try semi-auto init", "Green", true, false);
    slam.localization_stop();

    // semi auto init
    if(semi_auto_init_thread != NULL)
    {
        logger.write_log("[AUTO_INIT] thread already running.", "Orange", true, false);
        semi_auto_init_thread->join();
        semi_auto_init_thread = NULL;
    }

    semi_auto_init_thread = new std::thread(&SLAM_2D::semi_auto_init_start, &slam);
}

void MainWindow::bt_LocInitAuto()
{
    // auto init
}

void MainWindow::bt_LocStart()
{
    if(slam.is_slam)
    {
        printf("[MAIN] mapping already running\n");
        return;
    }

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

    slam.localization_stop();
    obsmap.clear();

    unimap.load_map(map_dir);
    all_update();
}

void MainWindow::bt_AddNode()
{
    unimap.add_node(pick, ui->cb_NodeType->currentText(), ui->te_NodeInfo->toPlainText());
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

void MainWindow::bt_EditNodeName()
{
    unimap.edit_node_name(pick);
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

void MainWindow::bt_AutoLink()
{
    unimap.add_link_auto();
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
    double dx = ui->spb_NodeMovementXY->value();

    if(ui->ckb_TransformEntireNode->isChecked())
    {
        std::vector<QString> nodes_id = unimap.get_nodes();
        for(auto& id: nodes_id)
        {
            NODE* node = unimap.get_node_by_id(id);
            if(node == nullptr)
            {
                continue;
            }

            Eigen::Matrix4d tf = node->tf;
            tf(0,3) += dx;
            unimap.edit_node_pos(id, tf);
        }
    }
    else if(selected_nodes.size() != 0 )
    {
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            QString id = node->id;
            Eigen::Matrix4d tf = node->tf;
            tf(0,3) += dx;
            unimap.edit_node_pos(id, tf);
        }
    }
    else
    {
        QString id = pick.cur_node;
        if(id == "")
        {
            return;
        }

        NODE* node = unimap.get_node_by_id(id);
        if(node == nullptr)
        {
            return;
        }

        Eigen::Matrix4d tf = node->tf;
        tf(0,3) += dx;
        unimap.edit_node_pos(id, tf);
    }

    topo_update();
    pick_update();
}

void MainWindow::bt_NodePoseYUp()
{
    double dy = ui->spb_NodeMovementXY->value();

    if(ui->ckb_TransformEntireNode->isChecked())
    {
        std::vector<QString> nodes_id = unimap.get_nodes();
        for(auto& id: nodes_id)
        {
            NODE* node = unimap.get_node_by_id(id);
            if(node == nullptr)
            {
                continue;
            }

            Eigen::Matrix4d tf = node->tf;
            tf(1,3) += dy;
            unimap.edit_node_pos(id, tf);
        }
    }
    else if(selected_nodes.size() != 0 )
    {
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            QString id = node->id;
            Eigen::Matrix4d tf = node->tf;
            tf(1,3) += dy;
            unimap.edit_node_pos(id, tf);
        }
    }
    else
    {
        QString id = pick.cur_node;
        if(id == "")
        {
            return;
        }

        NODE* node = unimap.get_node_by_id(id);
        if(node == nullptr)
        {
            return;
        }

        Eigen::Matrix4d tf = node->tf;
        tf(1,3) += dy;
        unimap.edit_node_pos(id, tf);
    }

    topo_update();
    pick_update();
}

void MainWindow::bt_NodePoseThUp()
{
    Eigen::Matrix4d tf1 = ZYX_to_TF(0, 0, 0, 0, 0, ui->spb_NodeMovementTh->value()*D2R);

    if(ui->ckb_TransformEntireNode->isChecked())
    {
        std::vector<QString> nodes_id = unimap.get_nodes();
        for(auto& id: nodes_id)
        {
            NODE* node = unimap.get_node_by_id(id);
            if(node == nullptr)
            {
                continue;
            }

            Eigen::Matrix4d tf0 = node->tf;
            Eigen::Matrix4d tf = tf0 * tf1;
            unimap.edit_node_pos(id, tf);
        }
    }
    else if(selected_nodes.size() != 0 )
    {
        double cnt = 0.0;
        Eigen::Vector3d center_pose(0, 0, 0);
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            cnt++;
            Eigen::Matrix4d tf0 = node->tf;
            Eigen::Vector3d pose0 = tf0.block(0,3,3,1);
            center_pose += pose0;
        }

        if(cnt == 0.0)
        {
            return;
        }

        center_pose /= cnt;
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            QString id = node->id;
            Eigen::Matrix4d tf0 = node->tf;
            Eigen::Vector3d pose0 = tf0.block(0,3,3,1);

            Eigen::Vector3d pose1 = pose0 - center_pose;
            pose1 = tf1.block(0,0,3,3) * pose1;

            Eigen::Vector3d pose = pose1 + center_pose;

            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            tf.block(0,0,3,3) = tf0.block(0,0,3,3) * tf1.block(0,0,3,3);
            tf.block(0,3,3,1) = pose;

            unimap.edit_node_pos(id, tf);
        }
    }
    else
    {
        QString id = pick.cur_node;
        if(id == "")
        {
            return;
        }

        NODE* node = unimap.get_node_by_id(id);
        if(node == nullptr)
        {
            return;
        }

        Eigen::Matrix4d tf0 = node->tf;
        Eigen::Matrix4d tf = tf0 * tf1;
        unimap.edit_node_pos(id, tf);
    }

    topo_update();
    pick_update();
}

void MainWindow::bt_NodePoseXDown()
{
    double dx = ui->spb_NodeMovementXY->value();

    if(ui->ckb_TransformEntireNode->isChecked())
    {
        std::vector<QString> nodes_id = unimap.get_nodes();
        for(auto& id: nodes_id)
        {
            NODE* node = unimap.get_node_by_id(id);
            if(node == nullptr)
            {
                continue;
            }

            Eigen::Matrix4d tf = node->tf;
            tf(0,3) -= dx;
            unimap.edit_node_pos(id, tf);
        }
    }
    else if(selected_nodes.size() != 0 )
    {
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            QString id = node->id;
            Eigen::Matrix4d tf = node->tf;
            tf(0,3) -= dx;
            unimap.edit_node_pos(id, tf);
        }
    }
    else
    {
        QString id = pick.cur_node;
        if(id == "")
        {
            return;
        }

        NODE* node = unimap.get_node_by_id(id);
        if(node == nullptr)
        {
            return;
        }

        Eigen::Matrix4d tf = node->tf;
        tf(0,3) -= dx;
        unimap.edit_node_pos(id, tf);
    }

    topo_update();
    pick_update();
}

void MainWindow::bt_NodePoseYDown()
{
    double dy = ui->spb_NodeMovementXY->value();

    if(ui->ckb_TransformEntireNode->isChecked())
    {
        std::vector<QString> nodes_id = unimap.get_nodes();
        for(auto& id: nodes_id)
        {
            NODE* node = unimap.get_node_by_id(id);
            if(node == nullptr)
            {
                continue;
            }

            Eigen::Matrix4d tf = node->tf;
            tf(1,3) -= dy;
            unimap.edit_node_pos(id, tf);
        }
    }
    else if(selected_nodes.size() != 0 )
    {
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            QString id = node->id;
            Eigen::Matrix4d tf = node->tf;
            tf(1,3) -= dy;
            unimap.edit_node_pos(id, tf);
        }
    }
    else
    {
        QString id = pick.cur_node;
        if(id == "")
        {
            return;
        }

        NODE* node = unimap.get_node_by_id(id);
        if(node == nullptr)
        {
            return;
        }

        Eigen::Matrix4d tf = node->tf;
        tf(1,3) -= dy;
        unimap.edit_node_pos(id, tf);
    }

    topo_update();
    pick_update();
}

void MainWindow::bt_NodePoseThDown()
{
    Eigen::Matrix4d tf1 = ZYX_to_TF(0, 0, 0, 0, 0, -ui->spb_NodeMovementTh->value()*D2R);

    if(ui->ckb_TransformEntireNode->isChecked())
    {
        std::vector<QString> nodes_id = unimap.get_nodes();
        for(auto& id: nodes_id)
        {
            NODE* node = unimap.get_node_by_id(id);
            if(node == nullptr)
            {
                continue;
            }

            Eigen::Matrix4d tf0 = node->tf;
            Eigen::Matrix4d tf = tf0 * tf1;
            unimap.edit_node_pos(id, tf);
        }
    }
    else if(selected_nodes.size() != 0 )
    {
        double cnt = 0.0;
        Eigen::Vector3d center_pose(0, 0, 0);
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            cnt++;
            Eigen::Matrix4d tf0 = node->tf;
            Eigen::Vector3d pose0 = tf0.block(0,3,3,1);
            center_pose += pose0;
        }

        if(cnt == 0.0)
        {
            return;
        }

        center_pose /= cnt;
        for(size_t p = 0; p < selected_nodes.size(); p++)
        {
            NODE* node = unimap.get_node_by_id(selected_nodes[p]);
            if(node == nullptr)
            {
                continue;
            }

            QString id = node->id;
            Eigen::Matrix4d tf0 = node->tf;
            Eigen::Vector3d pose0 = tf0.block(0,3,3,1);

            Eigen::Vector3d pose1 = pose0 - center_pose;
            pose1 = tf1.block(0,0,3,3) * pose1;

            Eigen::Vector3d pose = pose1 + center_pose;

            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            tf.block(0,0,3,3) = tf0.block(0,0,3,3) * tf1.block(0,0,3,3);
            tf.block(0,3,3,1) = pose;

            unimap.edit_node_pos(id, tf);
        }
    }
    else
    {
        QString id = pick.cur_node;
        if(id == "")
        {
            return;
        }

        NODE* node = unimap.get_node_by_id(id);
        if(node == nullptr)
        {
            return;
        }

        Eigen::Matrix4d tf0 = node->tf;
        Eigen::Matrix4d tf = tf0 * tf1;
        unimap.edit_node_pos(id, tf);
    }

    topo_update();
    pick_update();
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

    NODE* node0 = unimap.get_node_by_id(id0);
    NODE* node1 = unimap.get_node_by_id(id1);
    if(node0 == nullptr || node1 == nullptr)
    {
        return;
    }

    Eigen::Matrix4d tf0 = node0->tf;
    Eigen::Matrix4d tf1 = node1->tf;

    // if(tf0(0,3) > tf1(0,3))
    // {
    //     tf1(0,3) = tf0(0,3) + gap;
    // }
    // else
    // {
    //     tf1(0,3) = tf0(0,3) - gap;
    // }

    Eigen::Vector3d dir = tf0.block(0,0,3,1);
    Eigen::Vector3d _gap = dir * gap;

    tf1.block(0,3,3,1) = tf0.block(0,3,3,1) + _gap;

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

    NODE* node0 = unimap.get_node_by_id(id0);
    NODE* node1 = unimap.get_node_by_id(id1);
    if(node0 == nullptr || node1 == nullptr)
    {
        return;
    }

    Eigen::Matrix4d tf0 = node0->tf;
    Eigen::Matrix4d tf1 = node1->tf;

    // if(tf0(1,3) > tf1(1,3))
    // {
    //     tf1(1,3) = tf0(1,3) + gap;
    // }
    // else
    // {
    //     tf1(1,3) = tf0(1,3) - gap;
    // }

    Eigen::Vector3d dir = tf0.block(0,1,3,1);
    Eigen::Vector3d _gap = dir * gap;

    tf1.block(0,3,3,1) = tf0.block(0,3,3,1) + _gap;

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

    NODE* node0 = unimap.get_node_by_id(id0);
    NODE* node1 = unimap.get_node_by_id(id1);
    if(node0 == nullptr || node1 == nullptr)
    {
        return;
    }

    Eigen::Matrix4d tf0 = node0->tf;
    Eigen::Matrix4d tf1 = node1->tf;

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

    NODE* node0 = unimap.get_node_by_id(id0);
    NODE* node1 = unimap.get_node_by_id(id1);
    if(node0 == nullptr || node1 == nullptr)
    {
        return;
    }

    Eigen::Matrix4d tf0 = node0->tf;
    Eigen::Matrix4d tf1 = node1->tf;

    // align y
    tf1(1,3) = tf0(1,3);
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

    NODE* node0 = unimap.get_node_by_id(id0);
    NODE* node1 = unimap.get_node_by_id(id1);
    if(node0 == nullptr || node1 == nullptr)
    {
        return;
    }

    Eigen::Matrix4d tf0 = node0->tf;
    Eigen::Matrix4d tf1 = node1->tf;

    // align th
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

    // loop start
    slam.is_qa = true;
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
    slam.is_qa = false;
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

void MainWindow::bt_QuickAddNode()
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

    Eigen::Matrix4d cur_tf = slam.get_cur_tf();
    unimap.add_node(cur_tf, "GOAL");

    topo_update();
    printf("[QA] quick add node\n");
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

    if(ctrl.is_multi)
    {
        if(pick.cur_node != "")
        {
            ctrl.set_goal(pick.cur_node);
        }
    }
    else
    {
        // get goal
        Eigen::Matrix4d goal_tf = Eigen::Matrix4d::Identity();
        if(pick.cur_node != "")
        {
            NODE* node = unimap.get_node_by_id(pick.cur_node);
            if(node != NULL)
            {
                goal_tf = node->tf;
                printf("[MAIN] clicked goal, %s\n", pick.cur_node.toLocal8Bit().data());
            }
        }
        else
        {
            if(pick.last_btn == 1)
            {
                goal_tf = se2_to_TF(pick.r_pose);
                printf("[MAIN] clicked goal pose, %.2f, %.2f, %.2f\n", pick.r_pose[0], pick.r_pose[1], pick.r_pose[2]*R2D);
            }
            else
            {
                printf("[MAIN] goal empty\n");
                return;
            }
        }

        // pure pursuit
        ctrl.move_pp(goal_tf, 0);
    }
}

void MainWindow::bt_AutoMove2()
{

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

void MainWindow::slot_local_path_updated()
{
    is_local_path_update = true;
    is_local_path_update2 = true;
}

void MainWindow::slot_global_path_updated()
{
    is_global_path_update = true;
    is_global_path_update2 = true;
}

// for test
void MainWindow::bt_Test()
{
    std::vector<QString> path;
    path.push_back("N_0112");
    path.push_back("N_1729820377638");
    path.push_back("N_1729820437542");
    path.push_back("N_1729820446577");
    path.push_back("N_1729746747992");
    path.push_back("N_0106");
    path.push_back("N_1729746747992");
    path.push_back("N_1729820446577");
    path.push_back("N_1729820437542");
    path.push_back("N_1729820461736");
    ctrl.move_pp(path, 0);
}

void MainWindow::bt_TestLed()
{
    mobile.led(0, ui->spb_Led->value());
    printf("[MAIN] led test:%d\n", ui->spb_Led->value());
}

void MainWindow::ckb_TestDebug()
{
    ctrl.is_debug = ui->ckb_TestDebug->isChecked();
}

void MainWindow::bt_TestImgSave()
{
    cv::Mat cur_img0 = cam.get_img0();
    cv::Mat cur_img1 = cam.get_img1();

    if(cur_img0.empty())
    {
        printf("[MAIN] cur_img0 is empty. Skipping save operation.\n");
        return;
    }
    if(cur_img1.empty())
    {
        printf("[MAIN] cur_img1 is empty. Skipping save operation.\n");
        return;
    }

    QString home_dir = QDir::homePath();
    QString img_folder = home_dir + "/Pictures";

    QDir dir;
    if(!dir.exists(img_folder))
    {
        if(!dir.mkpath(img_folder))
        {
            printf("[MAIN] Failed to create image directory: %s\n", img_folder.toStdString().c_str());
            return;
        }
    }

    QString img_path0 = img_folder + "/img0.png";
    QString img_path1 = img_folder + "/img1.png";

    if(!cv::imwrite(img_path0.toStdString(), cur_img0))
    {
        printf("[MAIN] Failed to save img0.png\n");
    }
    else
    {
        printf("[MAIN] Saved img0.png at %s",img_path0.toStdString().c_str());
    }

    if(!cv::imwrite(img_path1.toStdString(), cur_img1))
    {
         printf("[MAIN] Failed to save img1.png\n");
    }
    else
    {
         printf("[MAIN] Saved img1.png at %s",img_path1.toStdString().c_str());
    }

    printf("[MAIN] Img Save test complete.\n");

}

// for obsmap
void MainWindow::bt_ObsClear()
{
    obsmap.clear();
    obs_update();
}

// for task
void MainWindow::ui_tasks_update()
{
    ui->lw_TaskList->clear();
    for(size_t i = 0; i < task.task_node_list.size(); i++)
    {
        ui->lw_TaskList->addItem(task.task_node_list[i]);
    }
    is_topo_update = true;
}

void MainWindow::bt_TaskAdd()
{
    if(pick.cur_node.isEmpty())
    {
        printf("[TASK] No selected node\n");
        return;
    }

    NODE* node = unimap.get_node_by_id(pick.cur_node);
    if(node == NULL)
    {
        printf("[TASK] Node null\n");
        return;
    }

    if(node->type == "GOAL" || node->type == "INIT")
    {
        task.add_task(node);
    }
    else
    {
        printf("[TASK] Unselectable node\n");
        return;
    }

    ui_tasks_update();
}

void MainWindow::bt_TaskDel()
{
    if(!ui->lw_TaskList->selectedItems().isEmpty())
    {
        // list delete
        QString id = ui->lw_TaskList->selectedItems().front()->text();
        NODE* node = unimap.get_node_by_id(id);
        if(node != NULL)
        {
            task.del_task(node);
        }
    }
    else
    {
        if(pick.cur_node.isEmpty())
        {
            printf("No selected node\n");
            return;
        }

        NODE* node = unimap.get_node_by_id(pick.cur_node);
        if(node == NULL)
        {
            printf("Node null\n");
            return;
        }

        task.del_task(node);
    }

    ui_tasks_update();
}

void MainWindow::bt_TaskSave()
{
    if(unimap.is_loaded == false || task.task_node_list.empty())
    {
        printf("no task list\n");
        return;
    }

    task.save_task(unimap.map_dir);
}

void MainWindow::bt_TaskLoad()
{
    // pcd map load
    if(unimap.is_loaded == false)
    {
        printf("no map\n");
        return;
    }

    QString path = QFileDialog::getOpenFileName(this, "Select dir", unimap.map_dir);
    QFileInfo file_info(path);

    QString file_name = file_info.fileName();
    if(!file_name.isNull() && unimap.map_dir == file_info.absolutePath()
            && file_info.baseName().contains("task_", Qt::CaseInsensitive)
            && file_info.suffix().compare("json", Qt::CaseInsensitive) == 0)
    {
        task.load_task(path);
    }
    else
    {
        printf("no task file\n");
        return;
    }

    ui_tasks_update();

}

void MainWindow::bt_TaskPlay()
{
    if(unimap.is_loaded == false || slam.is_loc == false || task.task_node_list.empty() || task.is_tasking)
    {
        printf("check again\n");
        return;
    }

    task.is_start = true;
    task.use_looping = ui->ckb_Looping->isChecked();
    printf("[TASK] use looping: %d\n", (int)task.use_looping);

    QString mode = ui->cb_TaskDrivingMode->currentText();
    #if defined (USE_SRV) || (USE_AMR_400) || (USE_AMR_400_LAKI)
    if(mode == "holonomic")
    {
        mode = "basic";
    }
    #endif
    task.play(mode);
}

void MainWindow::bt_TaskPause()
{
    task.pause();
}

void MainWindow::bt_TaskCancel()
{
    task.cancel();
}

// for fms
void MainWindow::bt_SendMap()
{
    if(unimap.is_loaded == false)
    {
        printf("[SEND_MAP] check map load\n");
        return;
    }

    QString program = "sshpass";
    QStringList arguments;
    arguments << "-p" << config.SERVER_PW
              << "rsync" << "-avz" << "--rsh=ssh -o StrictHostKeyChecking=no -p 22"
              << map_dir
              << QString("%1@%2:%3").arg(config.SERVER_ID, config.SERVER_IP, "~/maps/");

    QProcess process;
    process.start(program, arguments);
    if(!process.waitForStarted())
    {
        QMessageBox::information(this, "Send Map", "Failed to start process.");
        return;
    }

    process.waitForFinished();
    int exit_code = process.exitCode();
    if(process.exitStatus() == QProcess::NormalExit && exit_code == 0)
    {
        QMessageBox::information(this, "Send Map", "Send Successful.");
    }
    else
    {
        QString error_output = process.readAllStandardError();
        QMessageBox::critical(this, "Send Map", "Send Failed.\nReason:\n" + error_output);
    }
}

// comm
void MainWindow::comm_loop()
{
    int cnt = 0;

    std::string pipeline0 = "appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=30 ! video/x-h264,profile=baseline ! rtspclientsink location=rtsp://127.0.0.1:8554/cam0";
    std::string pipeline1 = "appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=30 ! video/x-h264,profile=baseline ! rtspclientsink location=rtsp://127.0.0.1:8554/cam1";

    const int send_w = 320;
    const int send_h = 240;

    cv::VideoWriter writer0;
    cv::VideoWriter writer1;

    printf("[COMM] loop start\n");
    while(comm_flag)
    {
        cnt++;

        // for 100ms loop
        if(cnt % 1 == 0)
        {
            if(cms.is_connected)
            {
                cms.send_status();
            }

            if(cui.is_connected)
            {
                Q_EMIT signal_send_status();
            }

            if(cfms.is_connected)
            {
                Q_EMIT signal_send_info();
            }

            if(config.USE_RTSP && config.USE_CAM)
            {
                // cam streaming
                if(cam.is_connected0)
                {
                    if(writer0.isOpened())
                    {
                        cv::Mat img = cam.get_img0();
                        if(!img.empty())
                        {
                            if(img.cols != send_w || img.rows != send_h)
                            {
                                cv::Mat _img;
                                cv::resize(img, _img, cv::Size(send_w, send_h));
                                writer0.write(_img);
                            }
                            else
                            {
                                writer0.write(img);
                            }
                        }
                    }
                }

                if(cam.is_connected1)
                {
                    if(writer1.isOpened())
                    {
                        cv::Mat img = cam.get_img1();
                        if(!img.empty())
                        {
                            if(img.cols != send_w || img.rows != send_h)
                            {
                                cv::Mat _img;
                                cv::resize(img, _img, cv::Size(send_w, send_h));
                                writer1.write(_img);
                            }
                            else
                            {
                                writer1.write(img);
                            }
                        }
                    }
                }
            }
        }

        // for 500ms loop
        if(cnt % 5 == 0)
        {
            if(cms.is_connected)
            {
                cms.send_mapping_cloud();

                if(is_global_path_update2)
                {
                    is_global_path_update2 = false;
                    cms.send_global_path();
                }

                if(is_local_path_update2)
                {
                    is_local_path_update2 = false;
                    cms.send_local_path();
                }
            }
        }

        // for 1000ms loop
        if(cnt % 10 == 0)
        {
            if(cms.is_connected)
            {
                cms.send_lidar();
            }

            // open video writer
            if(config.USE_RTSP && config.USE_CAM)
            {
                if(cam.is_connected0)
                {
                    if(!writer0.isOpened())
                    {
                        writer0.open(pipeline0, 0, (double)10, cv::Size(320,240), true);
                        logger.write_log("[COMM] cam0 rtsp writer try open");
                    }
                }

                if(cam.is_connected1)
                {
                    if(!writer1.isOpened())
                    {
                        writer1.open(pipeline1, 0, (double)10, cv::Size(320,240), true);
                        logger.write_log("[COMM] cam1 rtsp writer try open");
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[COMM] loop stop\n");
}

// watchdog
void MainWindow::watch_loop()
{
    int cnt = 0;
    int loc_fail_cnt = 0;
    double last_sync_time = 0;
    bool last_fms_connection = false;

    CPU_USAGE pre_cpu_usage;

    printf("[WATCHDOG] loop start\n");
    while(watch_flag)
    {
        cnt++;

        if(is_pressed_btn_c == true && is_pressed_btn_ctrl == true && contains_nodes.size() != 0)
        {
            temp_contains_nodes.clear();
            temp_contains_nodes = contains_nodes;
        }

        if(is_pressed_btn_v == true && is_pressed_btn_ctrl == true && temp_contains_nodes.size() != 0)
        {
            copy_contains_nodes.clear();
            for(size_t p=0; p < temp_contains_nodes.size(); p++)
            {
                NODE* node = unimap.get_node_by_id(temp_contains_nodes[p]);
                if(node == NULL)
                {
                    continue;
                }

                Eigen::Matrix4d tf = node->tf;
                QString type = node->type;

                // for click offset
                tf(0,3) += 0.5;
                tf(1,3) -= 0.5;

                QString id = unimap.add_node(tf, type);
                copy_contains_nodes.push_back(id);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if(copy_contains_nodes.size() != 0)
            {
                selected_nodes = copy_contains_nodes;
                contains_nodes.clear();
                temp_contains_nodes.clear();
                pick.pre_node = "";
                pick.cur_node = "";
                topo_update();
                pick_update();
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
                    if((ms.status_m0 == 0 || ms.status_m1 == 0) && ms.emo_state == 1 && ms.charge_state == 0)
                    {
                        mobile.motor_on();
                    }

                    if(ms.connection_m0 == 1 && ms.connection_m1 == 1 &&
                       ms.status_m0 == 1 && ms.status_m1 == 1 &&
                       ms.emo_state == 1 && ms.charge_state == 0)
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

            // check lidar
            if(lidar.is_connected_f)
            {
                if(lidar.is_synced_f == false)
                {
                    lidar.sync_f();
                    logger.write_log("[WATCH] try time sync, pc and front lidar");
                }
            }

            if(lidar.is_connected_b)
            {
                if(lidar.is_synced_b == false)
                {
                    lidar.sync_b();
                    logger.write_log("[WATCH] try time sync, pc and back lidar");
                }
            }

            // resync for time skew
            if(get_time() - last_sync_time > 1200.0 && ctrl.is_moving == false)
            {
                logger.write_log("[WATCH] resync all");

                if(mobile.is_connected)
                {
                    mobile.sync();
                }

                if(lidar.is_connected_f)
                {
                    lidar.sync_f();
                }

                if(lidar.is_connected_b)
                {
                    lidar.sync_b();
                }                

                last_sync_time = get_time();
            }

            // check loc
            if(slam.is_loc == false)
            {
                loc_fail_cnt = 0;
                slam.set_cur_loc_state("none");
                led_color = LED_MAGENTA;
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
                        led_color = LED_MAGENTA_BLINK;
                    }
                }
                else
                {
                    loc_fail_cnt = 0;
                    slam.set_cur_loc_state("good");
                }
            }

            // check multi
            if(last_fms_connection == true && cfms.is_connected == false)
            {
                printf("[WATCH] fms disconnect dectected\n");
            }
            last_fms_connection = cfms.is_connected;

            QString fms_info_str = "[FMS_INFO]\n" + config.SERVER_ID + "\n" +  config.SERVER_IP;
            ui->lb_FmsInfo->setText(fms_info_str);

            // check system info
            {
                MOBILE_STATUS ms = mobile.get_status();

                QString temp_str;
                QString power_str;
                QString cpu_usage_str;

                // check temperature
                {
                    int cpu_temp_sum = 0;
                    int cnt = 0;

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
                                    cnt++;
                                }
                            }
                        }
                    }

                    if(cnt != 0)
                    {
                        int pc_temp = cpu_temp_sum/cnt;
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
                    int cnt = 0;

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
                                    cnt++;
                                }
                            }
                        }
                    }

                    if(cnt != 0)
                    {
                        power_str.sprintf("[POWER] cpu:%.2f, m0:%.2f, m1:%.2f", (double)cpu_power_sum/cnt, (double)ms.cur_m0/10.0, (double)ms.cur_m1/10.0);
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

                QString system_info_str = "[SYSTEM_INFO]\n" + temp_str + "\n" + power_str + "\n" + cpu_usage_str;
                ui->lb_SystemInfo->setText(system_info_str);

                if(ui->ckb_RecordSystemInfo->isChecked())
                {
                    log_cnt++;
                    if(log_cnt >= 10)
                    {
                        log_cnt = 0;
                        system_logger.write_log(system_info_str, "White", true, true);
                    }
                }
            }

            // set led
            mobile.led(0, led_color);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[WATCHDOG] loop stop\n");
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
            printf("[JOG] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    printf("[JOG] loop stop\n");
}

// for plot loop
void MainWindow::map_plot()
{
    if(unimap.is_loaded == false)
    {
        return;
    }

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
}
void MainWindow::obs_plot()
{
    if(unimap.is_loaded == false)
    {
        return;
    }

    if(is_obs_update)
    {
        is_obs_update = false;

        cv::Mat obs_map;
        cv::Mat dyn_map;
        Eigen::Matrix4d obs_tf;
        obsmap.get_obs_map(obs_map, obs_tf);
        obsmap.get_dyn_map(dyn_map, obs_tf);

        std::vector<Eigen::Vector4d> plot_pts = obsmap.get_plot_pts();

        // plot grid map
        {
            cv::Mat plot_obs_map(obs_map.rows, obs_map.cols, CV_8UC3, cv::Scalar(0));
            obsmap.draw_robot(plot_obs_map, obs_tf);

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
                }
            }

            ui->lb_Screen1->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot_obs_map)));
            ui->lb_Screen1->setScaledContents(true);
            ui->lb_Screen1->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        }

        // plot obs pts
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < plot_pts.size(); p++)
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
void MainWindow::topo_plot()
{
    if(unimap.is_loaded == false)
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

                QString axis_id = id + "_axis";
                if(viewer->contains(axis_id.toStdString()))
                {
                    viewer->removeShape(axis_id.toStdString());
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
                for(size_t p = 0; p < unimap.nodes.size(); p++)
                {
                    QString id = unimap.nodes[p].id;
                    if(unimap.nodes[p].type == "ROUTE")
                    {
                        const double size = 0.15;
                        viewer->addCube(-size, size, -size, size, -size, size, 0.8, 0.8, 0.8, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = size - size2;
                        QString axis_id = id + "_axis";
                        viewer->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                    }
                    else if(unimap.nodes[p].type == "INIT")
                    {
                        viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                        0.0, 0.1, 0.0, 1.0, 1.0, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = config.ROBOT_SIZE_X[1] - size2;
                        QString axis_id = id + "_axis";
                        viewer->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                    }
                    else if(unimap.nodes[p].type == "GOAL")
                    {
                        viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                        0, 0.1, 0.5, 1.0, 0.0, id.toStdString());
                        viewer->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = config.ROBOT_SIZE_X[1] - size2;
                        QString axis_id = id + "_axis";
                        viewer->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
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
                        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = size - size2;
                        QString axis_id = id + "_axis";
                        viewer->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
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
                        if(node == NULL)
                        {
                            continue;
                        }

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
                // draw names
                for(size_t p = 0; p < unimap.nodes.size(); p++)
                {
                    if(unimap.nodes[p].type == "ZONE")
                    {
                        // plot text
                        QString id = unimap.nodes[p].id;
                        QString name = unimap.nodes[p].name;

                        QString text_id = id + "_text";
                        pcl::PointXYZ position;
                        position.x = unimap.nodes[p].tf(0,3);
                        position.y = unimap.nodes[p].tf(1,3);
                        position.z = unimap.nodes[p].tf(2,3) + 1.5;
                        viewer->addText3D(name.toStdString(), position, 0.2, 1.0, 0.0, 0.0, text_id.toStdString());

                        last_plot_names.push_back(text_id);
                    }
                }
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

        if(viewer->contains("pick_text"))
        {
            viewer->removeShape("pick_text");
        }

        if(viewer->contains("sel_cur"))
        {
            viewer->removeShape("sel_cur");
        }

        if(viewer->contains("sel_pre"))
        {
            viewer->removeShape("sel_pre");
        }        

        if(viewer->contains("text_cur"))
        {
            viewer->removeShape("text_cur");
        }

        if(viewer->contains("text_pre"))
        {
            viewer->removeShape("text_pre");
        }

        if(pick.cur_node != "")
        {
            NODE *node = unimap.get_node_by_id(pick.cur_node);
            if(node != NULL)
            {
                pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 1.0, 1.0, 1.0);
                viewer->addPolygonMesh(donut, "sel_cur");

                // plot text
                QString text = node->name;

                pcl::PointXYZ position;
                position.x = node->tf(0,3);
                position.y = node->tf(1,3);
                position.z = node->tf(2,3) + 1.0;
                viewer->addText3D(text.toStdString(), position, 0.2, 0.0, 0.0, 0.0, "text_cur");
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
            pcl::PointXYZ position;
            position.x = pick_tf(0,3);
            position.y = pick_tf(1,3);
            position.z = pick_tf(2,3) + 1.0;

            QString text;
            text.sprintf("%.2f/%.2f/%.2f", pick.r_pose[0], pick.r_pose[1], pick.r_pose[2]*R2D);
            viewer->addText3D(text.toStdString(), position, 0.3, 0.0, 0.0, 0.0, "pick_text");
        }
    }
}
void MainWindow::slam_plot()
{
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

            // plot keyframe pts
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
void MainWindow::loc_plot()
{
    if(slam.is_loc)
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

        // add cam pts
        TIME_PTS cam_scan0 = cam.get_scan0();
        Eigen::Matrix4d tf0 = cur_tpp.tf.inverse()*slam.get_best_tf(cam_scan0.t);

        for(size_t p = 0; p < cam_scan0.pts.size(); p+=4)
        {
            Eigen::Vector3d P = cam_scan0.pts[p];
            Eigen::Vector3d _P = tf0.block(0,0,3,3)*P + tf0.block(0,3,3,1);

            // set pos
            pcl::PointXYZRGB pt;
            pt.x = _P[0];
            pt.y = _P[1];
            pt.z = _P[2];
            pt.r = 128;
            pt.g = 128;
            pt.b = 0;
            cloud->push_back(pt);
        }

        TIME_PTS cam_scan1 = cam.get_scan1();
        Eigen::Matrix4d tf1 = cur_tpp.tf.inverse()*slam.get_best_tf(cam_scan1.t);
        for(size_t p = 0; p < cam_scan1.pts.size(); p+=4)
        {
            Eigen::Vector3d P = cam_scan1.pts[p];
            Eigen::Vector3d _P = tf1.block(0,0,3,3)*P + tf1.block(0,3,3,1);

            // set pos
            pcl::PointXYZRGB pt;
            pt.x = _P[0];
            pt.y = _P[1];
            pt.z = _P[2];
            pt.r = 0;
            pt.g = 128;
            pt.b = 128;
            cloud->push_back(pt);
        }

        if(!viewer->updatePointCloud(cloud, "raw_pts"))
        {
            viewer->addPointCloud(cloud, "raw_pts");
        }

        // pose update
        viewer->updatePointCloudPose("raw_pts",Eigen::Affine3f(cur_tpp.tf.cast<float>()));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, "raw_pts");
    }
}
void MainWindow::raw_plot()
{
    // plot mobile info
    if(mobile.is_connected)
    {
        // plot mobile pose
        ui->lb_MobilePoseInfo->setText(mobile.get_pose_text());

        // plot mobile status
        ui->lb_MobileStatusInfo->setText(mobile.get_status_text());
    }

    // plot slam info
    if(slam.is_slam || slam.is_loc)
    {
        ui->lb_SlamInfo->setText(slam.get_info_text());
    }

    // plot que info
    QString que_info_str;
    que_info_str.sprintf("[QUES]\nscan_q:%d, msg_q:%d, kfrm_q:%d\nplot_t:%.3f\nlidar_t:%.3f, %.3f",
                         (int)lidar.scan_que.unsafe_size(),
                         (int)mobile.msg_que.unsafe_size(),
                         (int)slam.kfrm_que.unsafe_size(),
                         (double)plot_proc_t,
                         (double)lidar.last_t_f, (double)lidar.last_t_b);
    ui->lb_QueInfo->setText(que_info_str);

    // plot auto info
    QString auto_info_str;
    auto_info_str.sprintf("[AUTO_INFO]\nfsm_state: %s\nis_moving: %s, is_pause: %s, obs: %s\nrequest: %s, multi: %s",
                          AUTO_FSM_STATE_STR[(int)ctrl.fsm_state].toLocal8Bit().data(),                          
                          (bool)ctrl.is_moving ? "1" : "0",
                          (bool)ctrl.is_pause ? "1" : "0",
                          ctrl.get_obs_condition().toLocal8Bit().data(),
                          cfms.get_multi_state().toLocal8Bit().data(),
                          ctrl.get_multi_req().toLocal8Bit().data());
    ui->lb_AutoInfo->setText(auto_info_str);

    // plot cam
    if(cam.is_connected0)
    {
        cv::Mat plot = cam.get_img0();
        if(!plot.empty())
        {
            ui->lb_Screen2->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
            ui->lb_Screen2->setScaledContents(true);
            ui->lb_Screen2->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        }
    }

    if(cam.is_connected1)
    {
        cv::Mat plot = cam.get_img1();
        if(!plot.empty())
        {
            ui->lb_Screen3->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
            ui->lb_Screen3->setScaledContents(true);
            ui->lb_Screen3->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        }
    }

    // if(cam.is_connected0)
    {
        if(config.USE_ARUCO)
        {
            cv::Mat plot = aruco.get_aruco0();
            if(!plot.empty())
            {
                ui->lb_Screen4->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
                ui->lb_Screen4->setScaledContents(true);
                ui->lb_Screen4->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
            }
        }
    }

    // if(cam.is_connected0)
    {
        if(config.USE_ARUCO)
        {
            cv::Mat plot = aruco.get_aruco1();
            if(!plot.empty())
            {
                ui->lb_Screen5->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot)));
                ui->lb_Screen5->setScaledContents(true);
                ui->lb_Screen5->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
            }
        }
    }

    if(!slam.is_slam && !slam.is_loc)
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

        // raw data
        std::vector<Eigen::Vector3d> cur_scan = lidar.get_cur_scan();
        std::vector<Eigen::Vector3d> cur_scan_b = blidar.get_cur_scan();
        std::vector<Eigen::Vector3d> cam_scan0 = cam.get_scan0().pts;
        std::vector<Eigen::Vector3d> cam_scan1 = cam.get_scan1().pts;

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

        for(size_t p = 0; p < cur_scan_b.size(); p++)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cur_scan_b[p][0];
            pt.y = cur_scan_b[p][1];
            pt.z = cur_scan_b[p][2];
            pt.r = 0;
            pt.g = 255;
            pt.b = 0;

            cloud->push_back(pt);
        }

        for(size_t p = 0; p < cam_scan0.size(); p+=4)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cam_scan0[p][0];
            pt.y = cam_scan0[p][1];
            pt.z = cam_scan0[p][2];
            pt.r = 128;
            pt.g = 128;
            pt.b = 0;

            cloud->push_back(pt);
        }

        for(size_t p = 0; p < cam_scan1.size(); p+=4)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cam_scan1[p][0];
            pt.y = cam_scan1[p][1];
            pt.z = cam_scan1[p][2];
            pt.r = 0;
            pt.g = 128;
            pt.b = 128;

            cloud->push_back(pt);
        }

        lidar.mtx.lock();
        std::vector<Eigen::Vector3d> cur_scan_outlier = lidar.cur_scan_outlier;
        lidar.mtx.unlock();

        for(size_t p = 0; p < cur_scan_outlier.size(); p++)
        {
            // set pos
            pcl::PointXYZRGB pt;
            pt.x = cur_scan_outlier[p][0];
            pt.y = cur_scan_outlier[p][1];
            pt.z = cur_scan_outlier[p][2];
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;

            cloud->push_back(pt);
        }

        /*
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
        */

        if(!viewer->updatePointCloud(cloud, "raw_pts"))
        {
            viewer->addPointCloud(cloud, "raw_pts");
        }

        // pose update
        Eigen::Matrix4d cur_tf = slam.get_cur_tf();
        viewer->updatePointCloudPose("raw_pts",Eigen::Affine3f(cur_tf.cast<float>()));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_PLOT_SIZE, "raw_pts");
    }
}
void MainWindow::ctrl_plot()
{
    // plot global path
    if(is_global_path_update)
    {
        is_global_path_update = false;

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
        if(global_path.pos.size() >= 4)
        {
            for(size_t p = 0; p < global_path.pos.size()-3; p+=3)
            {
                QString name;
                name.sprintf("global_path_%d_%d", (int)p, (int)p+3);

                Eigen::Vector3d P0 = global_path.pos[p];
                Eigen::Vector3d P1 = global_path.pos[p+3];

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
                if(p == local_path.pose.size()-1 || p % 25 == 0)
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
            ctrl.mtx.lock();
            Eigen::Vector3d cur_pos = ctrl.last_cur_pos;
            Eigen::Vector3d tgt_pos = ctrl.last_tgt_pos;
            Eigen::Vector3d local_goal = ctrl.last_local_goal;
            ctrl.mtx.unlock();

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
    Eigen::Matrix4d cur_tf = slam.get_cur_tf();

    // draw robot
    {
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
        text.sprintf("%.2f/%.2f/%.2f\n%s", (double)mobile.vx0, (double)mobile.vy0, (double)mobile.wz0*R2D, zone.toLocal8Bit().data());
        viewer->addText3D(text.toStdString(), position, 0.2, 0.0, 1.0, 1.0, "vel_text");

        // draw cur node
        if(viewer->contains("cur_tf_node"))
        {
            viewer->removeShape("cur_tf_node");
        }

        QString cur_node_id = unimap.get_node_id_edge(cur_tf.block(0,3,3,1));
        if(cur_node_id != "")
        {
            NODE *node = unimap.get_node_by_id(cur_node_id);
            if(node != NULL)
            {
                pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS*0.5, 0.05, node->tf, 0.0, 1.0, 1.0);
                viewer->addPolygonMesh(donut, "cur_tf_node");
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
}
void MainWindow::plot_loop()
{
    if(ui->ckb_PlotEnable->isChecked() == false)
    {
        return;
    }

    plot_timer.stop();

    double st_time = get_time();

    map_plot();
    obs_plot();
    topo_plot();
    pick_plot();

    slam_plot();
    loc_plot();
    raw_plot();
    ctrl_plot();

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
        Eigen::Matrix4d cur_tf = slam.get_cur_tf();
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
        printf("[MAIN] plot_loop, loop time drift: %f\n", (double)plot_proc_t);
    }

    plot_timer.start();
}

// for plot loop2
void MainWindow::map_plot2()
{
    if(unimap.is_loaded == false)
    {
        return;
    }

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
}
void MainWindow::topo_plot2()
{
    if(unimap.is_loaded == false)
    {
        return;
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
                    viewer2->removeShape(axis_id.toStdString());
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

        // remove names
        if(last_plot_names2.size() > 0)
        {
            for(size_t p = 0; p < last_plot_names2.size(); p++)
            {
                QString id = last_plot_names2[p];
                if(viewer2->contains(id.toStdString()))
                {
                    viewer2->removeShape(id.toStdString());
                }
            }
            last_plot_names2.clear();
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
                        const double size = 0.15;
                        viewer2->addCube(-size, size, -size, size, -size, size, 0.8, 0.8, 0.8, id.toStdString());
                        viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = size - size2;
                        QString axis_id = id + "_axis";
                        viewer2->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer2->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                    }
                    else if(unimap.nodes[p].type == "INIT")
                    {
                        viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                        0.0, 0.1, 0.0, 1.0, 1.0, id.toStdString());
                        viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = config.ROBOT_SIZE_X[1] - size2;
                        QString axis_id = id + "_axis";
                        viewer2->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer2->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                    }
                    else if(unimap.nodes[p].type == "GOAL")
                    {
                        viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                        config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                        0, 0.1, 0.5, 1.0, 0.0, id.toStdString());
                        viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = config.ROBOT_SIZE_X[1] - size2;
                        QString axis_id = id + "_axis";
                        viewer2->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer2->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                    }
                    else if(unimap.nodes[p].type == "OBS")
                    {
                        QString info = unimap.nodes[p].info;

                        NODE_INFO res;
                        if(parse_info(info, "SIZE", res))
                        {
                            viewer2->addCube(-res.sz[0]/2, res.sz[0]/2,
                                             -res.sz[1]/2, res.sz[1]/2,
                                             -res.sz[2]/2, res.sz[2]/2, 1.0, 0.0, 1.0, id.toStdString());

                            viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, id.toStdString());
                        }
                    }
                    else if(unimap.nodes[p].type == "ZONE")
                    {
                        QString info = unimap.nodes[p].info;

                        NODE_INFO res;
                        if(parse_info(info, "SIZE", res))
                        {
                            viewer2->addCube(-res.sz[0]/2, res.sz[0]/2,
                                             -res.sz[1]/2, res.sz[1]/2,
                                             -res.sz[2]/2, res.sz[2]/2, 0.5, 1.0, 0.0, id.toStdString());

                            viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, id.toStdString());
                        }
                    }
                    else if(unimap.nodes[p].type == "ARUCO")
                    {
                        const double size = 0.15;
                        viewer2->addCube(-size, size, -size, size, -size, size, 0.0, 1.0, 1.0, id.toStdString());
                        viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());

                        const double size2 = 0.05;
                        const double offset = size - size2;
                        QString axis_id = id + "_axis";
                        viewer2->addCube(-size2 + offset, size2 + offset, -size2, size2, -size2, size2, 1.0, 0.0, 0.0, axis_id.toStdString());
                        viewer2->updateShapePose(axis_id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
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
                        if(node == NULL)
                        {
                            continue;
                        }

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
                    if(unimap.nodes[p].type == "ZONE")
                    {
                        // plot text
                        QString id = unimap.nodes[p].id;
                        QString name = unimap.nodes[p].name;

                        QString text_id = id + "_text";
                        pcl::PointXYZ position;
                        position.x = unimap.nodes[p].tf(0,3);
                        position.y = unimap.nodes[p].tf(1,3);
                        position.z = unimap.nodes[p].tf(2,3) + 1.5;
                        viewer2->addText3D(name.toStdString(), position, 0.2, 1.0, 0.0, 0.0, text_id.toStdString());

                        last_plot_names2.push_back(text_id);
                    }
                }
            }
        }
    }
}
void MainWindow::pick_plot2()
{
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
                    str.sprintf("[pre_node]\nid: %s\nname: %s\ntype: %s\npos: %.3f, %.3f, %.2f\ninfo: %s",
                                pre_node->id.toLocal8Bit().data(),
                                pre_node->name.toLocal8Bit().data(),
                                pre_node->type.toLocal8Bit().data(),
                                pose[0], pose[1], pose[2]*R2D,
                                pre_node->info.toLocal8Bit().data());

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
                    str.sprintf("[cur_node]\nid: %s\nname: %s\ntype: %s\npos: %.3f, %.3f, %.2f\ninfo: %s",
                                cur_node->id.toLocal8Bit().data(),
                                cur_node->name.toLocal8Bit().data(),
                                cur_node->type.toLocal8Bit().data(),
                                pose[0], pose[1], pose[2]*R2D,
                                cur_node->info.toLocal8Bit().data());

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

        if(viewer2->contains("pick_text"))
        {
            viewer2->removeShape("pick_text");
        }

        if(viewer2->contains("sel_cur"))
        {
            viewer2->removeShape("sel_cur");
        }

        if(viewer2->contains("sel_pre"))
        {
            viewer2->removeShape("sel_pre");
        }

        if(viewer2->contains("text_cur"))
        {
            viewer2->removeShape("text_cur");
        }

        if(viewer2->contains("text_pre"))
        {
            viewer2->removeShape("text_pre");
        }

        // erase first
        if(last_plot_contains.size() > 0)
        {
            for(size_t p = 0; p < last_plot_contains.size(); p++)
            {
                QString contain_id = last_plot_contains[p] + "_contain";
                if(viewer2->contains(contain_id.toStdString()))
                {
                    viewer2->removeShape(contain_id.toStdString());
                }
            }
        }

        if(last_plot_copy.size() > 0)
        {
            for(size_t p = 0; p < last_plot_copy.size(); p++)
            {
                QString copy_id = last_plot_copy[p] + "_copy";
                if(viewer2->contains(copy_id.toStdString()))
                {
                    viewer2->removeShape(copy_id.toStdString());
                }
            }
        }

        if(viewer2->contains("contain_area"))
        {
            viewer2->removeShape("contain_area");
        }

        // different behavior each tab
        if(ui->annot_tab->tabText(ui->annot_tab->currentIndex()) == "EDIT_MAP")
        {
            if(pick.l_drag)
            {
                Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(pick.l_pt0[0], pick.l_pt0[1], 0));

                double val = ui->dsb_MapEraseSize->value();
                pcl::PolygonMesh donut = make_donut(val, 0.01, tf, 1.0, 0.0, 0.0, 0.5);
                viewer2->addPolygonMesh(donut, "pick_body");

                // erase
                unimap.set_cloud_mask(pick.l_pt0, val, 0);
                map_update();
            }
            else if(pick.r_drag)
            {
                Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(pick.r_pt0[0], pick.r_pt0[1], 0));

                double val = ui->dsb_MapEraseSize->value();
                pcl::PolygonMesh donut = make_donut(val, 0.01, tf, 0.0, 1.0, 0.0, 0.5);
                viewer2->addPolygonMesh(donut, "pick_body");

                // restore
                unimap.set_cloud_mask(pick.r_pt0, val, 1);
                map_update();
            }
        }
        else if(ui->annot_tab->tabText(ui->annot_tab->currentIndex()) == "EDIT_TOPO")
        {
            if(contains_nodes.size() != 0)
            {
                for(size_t p = 0; p < contains_nodes.size(); p++)
                {
                    NODE *node = unimap.get_node_by_id(contains_nodes[p]);
                    if(node != NULL)
                    {
                        QString id = node->id + "_contain";
                        pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 1.0, 1.0, 0.0);
                        viewer2->addPolygonMesh(donut, id.toStdString());
                        last_plot_contains.push_back(node->id);
                    }
                }
            }
            viewer2->addCube(lt_x, rb_x, rb_y, lt_y, -1.1, -1, 0.3, 0.3, 0.3, "contain_area");

            if(copy_contains_nodes.size() != 0)
            {
                for(size_t p = 0; p < copy_contains_nodes.size(); p++)
                {
                    NODE *node = unimap.get_node_by_id(copy_contains_nodes[p]);
                    if(node != NULL)
                    {
                        QString id = node->id + "_copy";
                        pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 1.0, 0.0, 0.5);
                        viewer2->addPolygonMesh(donut, id.toStdString());
                        last_plot_copy.push_back(node->id);
                    }
                }
            }

            // draw selection
            if(pick.pre_node != "")
            {
                NODE *node = unimap.get_node_by_id(pick.pre_node);
                if(node != NULL)
                {
                    if(node->type == "ZONE")
                    {
                        NODE_INFO res;
                        if(parse_info(node->info, "SIZE", res))
                        {
                            double r = std::min<double>(res.sz[0], res.sz[1])/2;
                            pcl::PolygonMesh donut = make_donut(r, 0.05, node->tf, 1.0, 0.0, 0.0);
                            viewer2->addPolygonMesh(donut, "sel_pre");
                        }
                    }
                    else
                    {
                        pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 1.0, 0.0, 0.0);
                        viewer2->addPolygonMesh(donut, "sel_pre");
                    }

                    // plot text
                    QString text = node->name;

                    pcl::PointXYZ position;
                    position.x = node->tf(0,3);
                    position.y = node->tf(1,3);
                    position.z = node->tf(2,3) + 1.0;
                    viewer2->addText3D(text.toStdString(), position, 0.2, 0.0, 0.0, 0.0, "text_pre");
                }
            }

            if(pick.cur_node != "")
            {
                NODE *node = unimap.get_node_by_id(pick.cur_node);
                if(node != NULL)
                {
                    if(node->type == "ZONE")
                    {
                        NODE_INFO res;
                        if(parse_info(node->info, "SIZE", res))
                        {
                            double r = std::min<double>(res.sz[0], res.sz[1])/2;
                            pcl::PolygonMesh donut = make_donut(r, 0.05, node->tf, 0.0, 1.0, 0.0);
                            viewer2->addPolygonMesh(donut, "sel_cur");
                        }
                    }
                    else
                    {
                        pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.05, node->tf, 0.0, 1.0, 0.0);
                        viewer2->addPolygonMesh(donut, "sel_cur");
                    }

                    // plot text
                    QString text = node->name;

                    pcl::PointXYZ position;
                    position.x = node->tf(0,3);
                    position.y = node->tf(1,3);
                    position.z = node->tf(2,3) + 1.0;
                    viewer2->addText3D(text.toStdString(), position, 0.2, 0.0, 0.0, 0.0, "text_cur");
                }
            }

            //if(pick.last_btn == 1)
            {
                Eigen::Matrix4d pick_tf = se2_to_TF(pick.r_pose);

                // draw pose
                viewer2->addCoordinateSystem(1.0, "O_pick");
                if(ui->cb_NodeType->currentText() == "ROUTE")
                {
                    viewer2->addSphere(pcl::PointXYZ(0, 0, 0), 0.15, 0.5, 0.5, 0.5, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(pick_tf.cast<float>()));
                }
                else if(ui->cb_NodeType->currentText() == "INIT")
                {
                    viewer2->addSphere(pcl::PointXYZ(0, 0, 0), 0.15, 0.5, 0.5, 0.5, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(pick_tf.cast<float>()));
                }
                else if(ui->cb_NodeType->currentText() == "GOAL")
                {
                    viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                                     config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                                     config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.5, 0.5, 0.5, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(pick_tf.cast<float>()));
                }
                else if(ui->cb_NodeType->currentText() == "OBS")
                {
                    double size_x = ui->spb_NodeSizeX->value();
                    double size_y = ui->spb_NodeSizeY->value();
                    double size_z = ui->spb_NodeSizeZ->value();

                    QString info;
                    info.sprintf("SIZE,%.2f,%.2f,%.2f\n", size_x, size_y, size_z);
                    ui->te_NodeInfo->setText(info);

                    viewer2->addCube(-size_x/2, size_x/2,
                                     -size_y/2, size_y/2,
                                     -size_z/2, size_z/2, 0.9, 0.9, 0.9, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(pick_tf.cast<float>()));
                }
                else if(ui->cb_NodeType->currentText() == "ZONE")
                {
                    double size_x = ui->spb_NodeSizeX->value();
                    double size_y = ui->spb_NodeSizeY->value();
                    double size_z = ui->spb_NodeSizeZ->value();

                    QString info;
                    info.sprintf("SIZE,%.2f,%.2f,%.2f\n", size_x, size_y, size_z);
                    ui->te_NodeInfo->setText(info);

                    viewer2->addCube(-size_x/2, size_x/2,
                                     -size_y/2, size_y/2,
                                     -size_z/2, size_z/2, 0.9, 0.9, 0.9, "pick_body");
                    viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "pick_body");
                    viewer2->updateShapePose("pick_body", Eigen::Affine3f(pick_tf.cast<float>()));
                }

                viewer2->updateCoordinateSystemPose("O_pick", Eigen::Affine3f(pick_tf.cast<float>()));

                // plot text
                pcl::PointXYZ position;
                position.x = pick_tf(0,3);
                position.y = pick_tf(1,3);
                position.z = pick_tf(2,3) + 1.0;

                QString text;
                text.sprintf("%.2f/%.2f/%.2f", pick.r_pose[0], pick.r_pose[1], pick.r_pose[2]*R2D);
                viewer2->addText3D(text.toStdString(), position, 0.3, 0.0, 0.0, 0.0, "pick_text");
            }
        }
    }
}
void MainWindow::loc_plot2()
{
    // get cur tf
    Eigen::Matrix4d cur_tf = slam.get_cur_tf();

    // draw robot
    {
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
}
void MainWindow::plot_loop2()
{
    if(ui->ckb_PlotEnable->isChecked() == false)
    {
        return;
    }

    plot_timer2.stop();

    map_plot2();
    topo_plot2();
    pick_plot2();
    loc_plot2();

    // rendering
    ui->qvtkWidget2->renderWindow()->Render();    

    plot_timer2.start();
}

void MainWindow::slot_write_log(QString user_log, QString color_code)
{
    if(color_code == "Red")
    {
        color_code = "red";
    }
    else if(color_code == "Green")
    {
        color_code = "green";
    }
    else if(color_code == "Orange")
    {
        color_code = "orange";
    }
    else if(color_code == "White")
    {
        color_code = "black";
    }

    color_code += "QLabel { color : " + color_code + "; }";

    ui->lb_LogInfo->setText(user_log);
    ui->lb_LogInfo->setStyleSheet(color_code);
}

