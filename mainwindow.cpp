#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , config(this)
    , plot_timer(this)
{
    ui->setupUi(this);

    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    connect(ui->bt_ConfigLoad, SIGNAL(clicked()), this, SLOT(bt_ConfigLoad()));
    connect(ui->bt_ConfigSave, SIGNAL(clicked()), this, SLOT(bt_ConfigSave()));

    // set plot window
    setup_vtk();

    // init modules
    init_modules();

    // start plot loop
    plot_timer.start(50);
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

void MainWindow::plot_loop()
{
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
