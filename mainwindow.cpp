#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , config(this)
    , mobile(this)
    , ui(new Ui::MainWindow)
    , plot_timer(this)
    , watchdog_timer(this)
{
    ui->setupUi(this);

    // timer
    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    connect(&watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdog_loop()));

    // config
    connect(ui->bt_ConfigLoad, SIGNAL(clicked()), this, SLOT(bt_ConfigLoad()));
    connect(ui->bt_ConfigSave, SIGNAL(clicked()), this, SLOT(bt_ConfigSave()));

    // motor
    connect(ui->bt_MotorInit, SIGNAL(clicked()), this, SLOT(bt_MotorInit()));

    // test
    connect(ui->bt_Sync, SIGNAL(clicked()), this, SLOT(bt_Sync()));

    // jog
    connect(ui->bt_JogF, SIGNAL(pressed()), this, SLOT(bt_JogF()));
    connect(ui->bt_JogB, SIGNAL(pressed()), this, SLOT(bt_JogB()));
    connect(ui->bt_JogL, SIGNAL(pressed()), this, SLOT(bt_JogL()));
    connect(ui->bt_JogR, SIGNAL(pressed()), this, SLOT(bt_JogR()));
    connect(ui->bt_JogF, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogB, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogL, SIGNAL(released()), this, SLOT(bt_JogReleased()));
    connect(ui->bt_JogR, SIGNAL(released()), this, SLOT(bt_JogReleased()));

    // set plot window
    setup_vtk();

    // init modules
    init_modules();

    // start plot loop
    plot_timer.start(50);
    watchdog_timer.start(1000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setup_vtk()
{
    // Set up the QVTK window
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    viewer->setBackgroundColor(1.0, 1.0, 1.0);
    viewer->resetCamera();

    // init drawing    
    viewer->addCoordinateSystem(1.0, "O_global");
    ui->qvtkWidget->renderWindow()->Render();

    // install event filter
    ui->qvtkWidget->installEventFilter(this);
    ui->qvtkWidget->setMouseTracking(true);
}

void MainWindow::draw_picking(Eigen::Vector3d pose)
{
    Eigen::Matrix4d tf = ZYX_to_TF(pose[0], pose[1], 0, 0, 0, pose[2]);

    // draw axis
    if(viewer->contains("picking_axis"))
    {
        viewer->removeCoordinateSystem("picking_axis");
    }
    viewer->addCoordinateSystem(1.0, "picking_axis");
    viewer->updateCoordinateSystemPose("picking_axis", Eigen::Affine3f(tf.cast<float>()));

    // draw body
    if(viewer->contains("picking_body"))
    {
        viewer->removeShape("picking_body");
    }
    viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                    config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                    config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.75, 0.75, 0.75, "picking_body");
    viewer->updateShapePose("picking_body", Eigen::Affine3f(tf.cast<float>()));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "picking_body");
}

bool MainWindow::eventFilter(QObject *object, QEvent *ev)
{
    if(object == ui->qvtkWidget)
    {
        // keyboard event
        if(ev->type() == QEvent::KeyPress)
        {
            return true;
        }
        else if(ev->type() == QEvent::KeyRelease)
        {
            return true;
        }

        // mouse event
        if(ev->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent* me = static_cast<QMouseEvent*>(ev);
            if(me->button() == Qt::LeftButton)
            {
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
                picking_ray(x, y, w, h, ray_center, ray_direction);

                Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                pick.r_pt0 = pt;
                pick.r_drag = true;

                // draw picking point
                if(viewer->contains("picking_point"))
                {
                    viewer->removeShape("picking_point");
                }
                viewer->addSphere(pcl::PointXYZ(pt[0], pt[1], pt[2]), 0.05, 1.0, 0.0, 0.0, "picking_point");

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
                double w = ui->qvtkWidget->size().width();
                double h = ui->qvtkWidget->size().height();

                Eigen::Vector3d ray_center;
                Eigen::Vector3d ray_direction;
                picking_ray(x, y, w, h, ray_center, ray_direction);

                Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                pick.r_pt1 = pt;

                // draw guide line
                if(viewer->contains("picking_line"))
                {
                    viewer->removeShape("picking_line");
                }
                viewer->addLine(pcl::PointXYZ(pick.r_pt0[0], pick.r_pt0[1], pick.r_pt0[2]),
                                pcl::PointXYZ(pick.r_pt1[0], pick.r_pt1[1], pick.r_pt1[2]), 1.0, 0, 0, "picking_line");

                // calc pose
                pick.r_pose[0] = pick.r_pt0[0];
                pick.r_pose[1] = pick.r_pt0[1];
                pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                // plot picking
                draw_picking(pick.r_pose);

                return true;
            }
        }

        if(ev->type() == QEvent::MouseButtonRelease)
        {
            QMouseEvent* me = static_cast<QMouseEvent*>(ev);
            if(me->button() == Qt::LeftButton)
            {
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
                picking_ray(x, y, w, h, ray_center, ray_direction);

                Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                pick.r_pt1 = pt;
                pick.r_drag = false;

                // remove guide line
                if(viewer->contains("picking_line"))
                {
                    viewer->removeShape("picking_line");
                }

                // calc pose
                pick.r_pose[0] = pick.r_pt0[0];
                pick.r_pose[1] = pick.r_pt0[1];
                pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                // plot picking
                draw_picking(pick.r_pose);

                return true;
            }
        }
    }

    return QWidget::eventFilter(object, ev);
}

void MainWindow::picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir)
{
    // ray casting
    std::vector<pcl::visualization::Camera> cams;
    viewer->getCameras(cams);

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

void MainWindow::init_modules()
{
    // config module init
    config.config_path = QCoreApplication::applicationDirPath() + "/config.json";
    config.ui_table = ui->tw_Config;

    config.load();
    config.config_to_ui();

    // mobile module init
    mobile.config = &config;
    mobile.open();

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

void MainWindow::watchdog_loop()
{
    // check mobile
    if(mobile.is_connected)
    {
        if(mobile.offset_t == 0)
        {
            printf("[WATCH] try time sync, pc and mobile\n");
            mobile.sync();
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


    // check camera


    // check loc


}

void MainWindow::plot_loop()
{
    // plot mobile info
    if(mobile.is_connected)
    {
        MOBILE_POSE mobile_pose = mobile.get_pose();
        QString mobile_pose_str;
        mobile_pose_str.sprintf("[MOBILE_POSE]\nt:%.3f, msg_q:%d\npos:%.2f,%.2f,%.2f\nvel:%.2f,%.2f,%.2f\nvw:%.2f,%.2f", mobile_pose.t, (int)mobile.msg_que.unsafe_size(),
                                mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
                                mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D,
                                mobile_pose.vw[0], mobile_pose.vw[1]*R2D);
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
        Eigen::Vector3d mobile_rpy = mobile.get_rpy();
        QString mobile_rpy_str;
        mobile_rpy_str.sprintf("[IMU_FILTERED]\nrx:%.3f, ry:%.3f, rz:%.3f\n", mobile_rpy[0]*R2D, mobile_rpy[1]*R2D, mobile_rpy[2]*R2D);
        ui->lb_ImuInfo->setText(mobile_rpy_str);
    }


    // cam control
    if(ui->cb_ViewType->currentText() == "FREE")
    {

    }
    else if(ui->cb_ViewType->currentText() == "FOLLOW")
    {

    }

    // draw cursor

    // rendering
    ui->qvtkWidget->renderWindow()->Render();
}
