#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , config(this)
    , mobile(this)
    , lidar(this)
    , slam(this)
    , unimap(this)
    , ui(new Ui::MainWindow)
    , plot_timer(this)
    , plot_timer2(this)
    , watchdog_timer(this)
{
    ui->setupUi(this);

    // timer
    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    connect(&plot_timer2, SIGNAL(timeout()), this, SLOT(plot_loop2()));
    connect(&watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdog_loop()));

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

    // check box
    connect(ui->ckb_PlotKfrm, SIGNAL(stateChanged(int)), this, SLOT(ckb_PlotKfrm()));

    // mapping
    connect(ui->bt_MapBuild, SIGNAL(clicked()), this, SLOT(bt_MapBuild()));
    connect(ui->bt_MapStop, SIGNAL(clicked()), this, SLOT(bt_MapStop()));
    connect(ui->bt_MapSave, SIGNAL(clicked()), this, SLOT(bt_MapSave()));
    connect(ui->bt_MapLoad, SIGNAL(clicked()), this, SLOT(bt_MapLoad()));

    // localization
    connect(ui->bt_LocInit, SIGNAL(clicked()), this, SLOT(bt_LocInit()));
    connect(ui->bt_LocInit2, SIGNAL(clicked()), this, SLOT(bt_LocInit2()));
    connect(ui->bt_LocStart, SIGNAL(clicked()), this, SLOT(bt_LocStart()));
    connect(ui->bt_LocStop, SIGNAL(clicked()), this, SLOT(bt_LocStop()));

    // annotation
    connect(ui->bt_MapEditLoad, SIGNAL(clicked()), this, SLOT(bt_MapEditLoad()));
    connect(ui->bt_MapEditSave, SIGNAL(clicked()), this, SLOT(bt_MapEditSave()));
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

    connect(ui->spb_MapPointSize, SIGNAL(valueChanged(int)), this, SLOT(updateMap(int)));
    connect(ui->ckb_PlotAreas, SIGNAL(stateChanged(int)), this, SLOT(updateMap(int)));
    connect(ui->ckb_PlotEdges, SIGNAL(stateChanged(int)), this, SLOT(updateTopo(int)));
    connect(ui->ckb_PlotNames, SIGNAL(stateChanged(int)), this, SLOT(updateTopo(int)));
    connect(ui->ckb_PlotNodes, SIGNAL(stateChanged(int)), this, SLOT(updateTopo(int)));

    // set plot window
    setup_vtk();

    // init modules
    init_modules();

    // start plot loop
    plot_timer.start(50);
    plot_timer2.start(50);
    watchdog_timer.start(1000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

pcl::PolygonMesh MainWindow::make_donut(double donut_radius, double tube_radius, Eigen::Matrix4d tf, double r, double g, double b, double a, int num_segments)
{
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.width = num_segments * num_segments;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int i = 0; i < num_segments; ++i)
    {
        double theta = 2.0 * M_PI * i / num_segments;
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);

        for (int j = 0; j < num_segments; ++j)
        {
            double phi = 2.0 * M_PI * j / num_segments;
            double cos_phi = std::cos(phi);
            double sin_phi = std::sin(phi);

            pcl::PointXYZRGBA point;
            point.x = (donut_radius + tube_radius * cos_phi) * cos_theta;
            point.y = (donut_radius + tube_radius * cos_phi) * sin_theta;
            point.z = tube_radius * sin_phi;
            point.r = static_cast<uint8_t>(r * 255);
            point.g = static_cast<uint8_t>(g * 255);
            point.b = static_cast<uint8_t>(b * 255);
            point.a = static_cast<uint8_t>(a * 255);

            cloud.points[i * num_segments + j] = point;
        }
    }

    pcl::transformPointCloud(cloud, cloud, tf);
    pcl::toPCLPointCloud2(cloud, mesh.cloud);

    mesh.polygons.resize(num_segments * num_segments);
    for (int i = 0; i < num_segments; ++i)
    {
        for (int j = 0; j < num_segments; ++j)
        {
            pcl::Vertices vertices;
            vertices.vertices.push_back((i * num_segments + j) % (num_segments * num_segments));
            vertices.vertices.push_back(((i + 1) * num_segments + j) % (num_segments * num_segments));
            vertices.vertices.push_back(((i + 1) * num_segments + (j + 1)) % (num_segments * num_segments));
            vertices.vertices.push_back((i * num_segments + (j + 1)) % (num_segments * num_segments));
            mesh.polygons[i * num_segments + j] = vertices;
        }
    }

    return mesh;
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
        ui->qvtkWidget2->SetRenderWindow(viewer2->getRenderWindow());
        viewer2->setupInteractor(ui->qvtkWidget2->GetInteractor(), ui->qvtkWidget2->GetRenderWindow());
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

void MainWindow::draw_picking(Eigen::Vector3d pose)
{
    Eigen::Matrix4d tf = se2_to_TF(pose);

    if(ui->main_tab->currentIndex() == TAB_SLAM)
    {
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

    else if(ui->main_tab->currentIndex() == TAB_ANNOTATION)
    {
        // draw axis
        if(viewer2->contains("picking_axis"))
        {
            viewer2->removeCoordinateSystem("picking_axis");
        }
        viewer2->addCoordinateSystem(1.0, "picking_axis");
        viewer2->updateCoordinateSystemPose("picking_axis", Eigen::Affine3f(tf.cast<float>()));

        // draw body
        if(viewer2->contains("picking_body"))
        {
            viewer2->removeShape("picking_body");
        }
        viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                         config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                         config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.75, 0.75, 0.75, "picking_body");
        viewer2->updateShapePose("picking_body", Eigen::Affine3f(tf.cast<float>()));
        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "picking_body");
    }
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

    else if(object == ui->qvtkWidget2)
    {
        if(ui->tabAnnotation->currentIndex() == TAB_MAP)
        {
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
                    return true;
                }
            }

            if(ev->type() == QEvent::MouseMove)
            {
                QMouseEvent* me = static_cast<QMouseEvent*>(ev);

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

                }
            }
        }
        else if(ui->tabAnnotation->currentIndex() == TAB_TOPO)
        {
            // keyboard event
            if(ev->type() == QEvent::KeyPress)
            {
                QKeyEvent* ke = static_cast<QKeyEvent*>(ev);
                if(!ke->isAutoRepeat()) // 키 반복이 아닐 때만 처리
                {
                    switch(ke->key())
                    {
                        case Qt::Key_N:
                            unimap.add_node(pick, ui->cb_NodeType->currentText());
                            break;
                        case Qt::Key_L:
                            unimap.add_link1(pick);
                            break;
                        case Qt::Key_B:
                            unimap.add_link2(pick);
                            break;
                        case Qt::Key_E:
                            unimap.edit_node_pos(pick);
                            break;
                        case Qt::Key_T:
                            unimap.edit_node_type(pick, ui->cb_NodeType->currentText());
                            break;
                        default:
                            break;
                    }

                    is_topo_update = true;
                    return true;
                }
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
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));

                    pick.pre_node = pick.cur_node;

                    NODE* pre_node = unimap.get_node_by_id(pick.pre_node);
                    if(pre_node != NULL)
                    {
                        QString str = "pre_node: " + pre_node->id;
                        ui->lb_PreNode->setText(str);
                    }

                    pick.cur_node = unimap.get_node_id(pt);
                    NODE* node = unimap.get_node_by_id(pick.cur_node);
                    if(node != NULL)
                    {
                        Eigen::Matrix3d R = node->tf.block(0,0,3,3);
                        Eigen::Vector3d euler = R.eulerAngles(2,1,0);
                        double th = euler[0];

                        Eigen::Vector3d t = node->tf.block(0,3,3,1);
                        pt = Eigen::Vector3d(t[0], t[1], th);

                        ui->le_CurNodeId->setText(node->id);
                        ui->le_CurNodeName->setText(node->name);
                        ui->le_CurNodeType->setText(node->type);
                        ui->le_CurNodeInfo->setText(node->info);
                    }

                    pick.r_pt0 = pt;
                    pick.r_drag = true;

                    // draw picking point
                    if(viewer2->contains("picking_point"))
                    {
                        viewer2->removeShape("picking_point");
                    }
                    viewer2->addSphere(pcl::PointXYZ(pt[0], pt[1],
                            pt[2]), 0.05, 1.0, 0.0, 0.0, "picking_point");

                    ui->le_CurNodeX->setText(QString::number(pt[0]));
                    ui->le_CurNodeY->setText(QString::number(pt[1]));
                    ui->le_CurNodeTh->setText(QString::number(pt[2]*R2D));

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
                    picking_ray(x, y, w, h, ray_center, ray_direction);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt1 = pt;

                    // draw guide line
                    if(viewer2->contains("picking_line"))
                    {
                        viewer2->removeShape("picking_line");
                    }
                    viewer2->addLine(pcl::PointXYZ(pick.r_pt0[0], pick.r_pt0[1], pick.r_pt0[2]),
                                    pcl::PointXYZ(pick.r_pt1[0], pick.r_pt1[1], pick.r_pt1[2]), 1.0, 0, 0, "picking_line");

                    // calc pose
                    pick.r_pose[0] = pick.r_pt0[0];
                    pick.r_pose[1] = pick.r_pt0[1];
                    pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                    // plot picking
                    draw_picking(pick.r_pose);

                    NODE* node = unimap.get_node_by_id(pick.cur_node);
                    if(node == NULL)
                    {
                        ui->le_CurNodeTh->setText(QString::number(pick.r_pose[2]*R2D));
                    }

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
                    double w = ui->qvtkWidget2->size().width();
                    double h = ui->qvtkWidget2->size().height();

                    Eigen::Vector3d ray_center;
                    Eigen::Vector3d ray_direction;
                    picking_ray(x, y, w, h, ray_center, ray_direction);

                    Eigen::Vector3d pt = ray_intersection(ray_center, ray_direction, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
                    pick.r_pt1 = pt;
                    pick.r_drag = false;

                    // remove guide line
                    if(viewer2->contains("picking_line"))
                    {
                        viewer2->removeShape("picking_line");
                    }

                    // calc pose
                    pick.r_pose[0] = pick.r_pt0[0];
                    pick.r_pose[1] = pick.r_pt0[1];
                    pick.r_pose[2] = std::atan2(pick.r_pt1[1] - pick.r_pt0[1], pick.r_pt1[0] - pick.r_pt0[0]);

                    // plot picking
                    draw_picking(pick.r_pose);

                    NODE* node = unimap.get_node_by_id(pick.cur_node);
                    if(node == NULL)
                    {
                        ui->le_CurNodeTh->setText(QString::number(pick.r_pose[2]*R2D));
                    }

                    return true;
                }
            }
        }
    }

    return QWidget::eventFilter(object, ev);
}

void MainWindow::picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir)
{
    // ray casting
    std::vector<pcl::visualization::Camera> cams;
    if(ui->main_tab->currentIndex() == TAB_SLAM)
    {
        viewer->getCameras(cams);
    }
    else if(ui->main_tab->currentIndex() == TAB_ANNOTATION)
    {
        viewer2->getCameras(cams);
    }

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

    // log module init
    logger.log_path = QCoreApplication::applicationDirPath() + "/snlog/";
    logger.init();

    // mobile module init
    mobile.config = &config;
    mobile.logger = &logger;
    mobile.open();

    // lidar module init
    lidar.config = &config;
    lidar.logger = &logger;
    lidar.open(&mobile);

    // slam module init
    slam.config = &config;
    slam.logger = &logger;
    slam.mobile = &mobile;
    slam.lidar = &lidar;
    slam.unimap = &unimap;

    // unimap module init
    unimap.config = &config;
    unimap.logger = &logger;
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

void MainWindow::ckb_PlotKfrm()
{
    if(ui->ckb_PlotKfrm->isChecked())
    {
        int num = slam.kfrm_storage.size();
        for(int p = 0; p < num; p++)
        {
            slam.kfrm_update_que.push(p);
        }
    }
}

void MainWindow::bt_MapBuild()
{
    // auto generation dir path
    QString _map_dir = QDir::homePath() + "/maps/" + get_time_str();
    QDir().mkpath(_map_dir);
    map_dir = _map_dir;

    // mapping start
    slam.mapping_start();
}

void MainWindow::bt_MapStop()
{
    slam.mapping_stop();
}

void MainWindow::bt_MapSave()
{
    // check
    if(map_dir == "")
    {
        printf("no map_dir\n");
        return;
    }

    // stop first
    slam.mapping_stop();

    // check kfrm
    if(slam.kfrm_storage.size() == 0)
    {
        printf("no keyframe\n");
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

        printf("convert: %d/%d ..\n", (int)(p+1), (int)slam.kfrm_storage.size());
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
        printf("%s saved\n", cloud_csv_path.toLocal8Bit().data());
    }
}

void MainWindow::bt_MapLoad()
{
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    if(!path.isNull())
    {
        map_dir = path;
        unimap.load_map(path);
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

void MainWindow::bt_MapEditLoad()
{
    // pcd map load
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    if(!path.isNull())
    {
        map_dir = path;
        unimap.load_map(path);

        is_map_update = true;
        is_topo_update = true;

        ui->lb_MapEditPath->setText(map_dir);
        ui->bt_MapLoad->setStyleSheet("background-color: rgb(0,255,0);");
    }
}

void MainWindow::bt_MapEditSave()
{
    unimap.save_map();
    unimap.save_annotation();
}

void MainWindow::bt_AddNode()
{
    unimap.add_node(pick, ui->cb_NodeType->currentText());

    // for topology update
    is_topo_update = true;
}

void MainWindow::bt_EditNodePos()
{
    unimap.edit_node_pos(pick);

    // for topology update
    is_topo_update = true;
}

void MainWindow::bt_EditNodeType()
{
    QString type = ui->cb_NodeType->currentText();
    unimap.edit_node_type(pick, type);

    // for topology update
    is_topo_update = true;
}

void MainWindow::bt_EditNodeInfo()
{
    unimap.edit_node_info(pick, ui->te_NodeInfo->toPlainText());
}

void MainWindow::bt_AddLink1()
{
    unimap.add_link1(pick);

    // for topology update
    is_topo_update = true;
}

void MainWindow::bt_AddLink2()
{
    unimap.add_link2(pick);

    // for topology update
    is_topo_update = true;
}

void MainWindow::bt_ClearTopo()
{
    if(unimap.is_loaded == false)
    {
        return;
    }

    unimap.clear_nodes();

    // for topology update
    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
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

    is_topo_update = true;
}

void MainWindow::updateMap(int val)
{
    is_map_update = true;
}

void MainWindow::updateTopo(int val)
{
    is_topo_update = true;
}

void MainWindow::watchdog_loop()
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


}

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

    // plot que info
    QString que_info_str;
    que_info_str.sprintf("raw_q:%d, raw_q_ex:%d, scan_q:%d\nl2c_q:%d", (int)lidar.raw_que_f.unsafe_size(), (int)lidar.raw_que_b.unsafe_size(), (int)lidar.scan_que.unsafe_size(), (int)mobile.msg_que.unsafe_size());
    ui->lb_QueInfo->setText(que_info_str);

    // plot map data
    if(unimap.is_loaded)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < unimap.kdtree_cloud.pts.size(); p++)
        {
            PT_XYZR _pt = unimap.kdtree_cloud.pts[p];

            pcl::PointXYZRGB pt;
            pt.x = _pt.x;
            pt.y = _pt.y;
            pt.z = _pt.z;

            double reflect = std::sqrt(_pt.r/255);
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



    // plot cloud data
    if(slam.is_slam)
    {
        // mapping
        if(slam.mtx.try_lock())
        {
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
                    if(slam.live_cloud.pts[p].do_cnt < config.SLAM_DO_ACCUM_NUM)
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
            if(ui->ckb_PlotKfrm->isChecked())
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
                            if(kfrm.pts[p].do_cnt < config.SLAM_DO_ACCUM_NUM)
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

                            plot_kfrm_names.push_back(name);
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
            else
            {
                // remove all keyframes
                if(plot_kfrm_names.size() > 0)
                {
                    for(size_t p = 0; p < plot_kfrm_names.size(); p++)
                    {
                        QString name = plot_kfrm_names[p];
                        if(viewer->contains(name.toStdString()))
                        {
                            viewer->removeShape(name.toStdString());
                        }
                    }
                }
            }

            slam.mtx.unlock();
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

    }
    else if(slam.is_loc)
    {
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

        // draw robot
        {
            // draw axis
            if(viewer->contains("robot_axis"))
            {
                viewer->removeCoordinateSystem("robot_axis");
            }
            viewer->addCoordinateSystem(1.0, "robot_axis");
            viewer->updateCoordinateSystemPose("robot_axis", Eigen::Affine3f(cur_tpp.tf.cast<float>()));

            // draw body
            if(viewer->contains("robot_body"))
            {
                viewer->removeShape("robot_body");
            }
            viewer->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                            config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                            config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 0.0, 0.5, 1.0, "robot_body");
            viewer->updateShapePose("robot_body", Eigen::Affine3f(cur_tpp.tf.cast<float>()));
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "robot_body");
        }
    }
    else
    {
        // get tf
        Eigen::Matrix4d cur_tf = slam.get_cur_tf();

        // raw data
        std::vector<Eigen::Vector3d> cur_scan_f = lidar.get_cur_scan_f();
        std::vector<Eigen::Vector3d> cur_scan_b = lidar.get_cur_scan_b();

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
    if(!unimap.is_loaded)
    {
        return;
    }

    // map plot
    if(is_map_update)
    {
        // clear flag
        is_map_update = false;

        // plot
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(size_t p = 0; p < unimap.kdtree_cloud.pts.size(); p++)
        {
            PT_XYZR xyzr = unimap.kdtree_cloud.pts[p];

            pcl::PointXYZRGB pt;
            pt.x = xyzr.x;
            pt.y = xyzr.y;
            pt.z = xyzr.z;

            pt.r = 0;
            pt.g = 0;
            pt.b = 0;

            cloud->push_back(pt);
        }

        if(!viewer2->updatePointCloud(cloud, "map_pts"))
        {
            viewer2->addPointCloud(cloud, "map_pts");
        }

        // point size
        viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->spb_MapPointSize->value(), "map_pts");
    }

    // plot topology
    if(is_topo_update)
    {
        // clear flag
        is_topo_update = false;

        // erase first
        {
            // remove nodes
            if(last_plot_nodes.size() > 0)
            {
                for(size_t p = 0; p < last_plot_nodes.size(); p++)
                {
                    QString id = last_plot_nodes[p];
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
                last_plot_nodes.clear();
            }

            // remove edges
            if(last_plot_edges.size() > 0)
            {
                for(size_t p = 0; p < last_plot_edges.size(); p++)
                {
                    QString id = last_plot_edges[p];
                    if(viewer2->contains(id.toStdString()))
                    {
                        viewer2->removeShape(id.toStdString());
                    }
                }
                last_plot_edges.clear();
            }
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

                        // pose_to_tf
                        viewer2->updateShapePose(id.toStdString(), Eigen::Affine3f(unimap.nodes[p].tf.cast<float>()));
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id.toStdString());
                    }

                    // for erase
                    last_plot_nodes.push_back(id);
                }
            }

            if(ui->ckb_PlotNames->isChecked())
            {
                // draw nodes
                for(size_t p = 0; p < unimap.nodes.size(); p++)
                {
                    QString id = unimap.nodes[p].id;

                    // plot text
                    QString text_id = id + "_text";
                    pcl::PointXYZ position;
                    position.x = unimap.nodes[p].tf(0,3);
                    position.y = unimap.nodes[p].tf(1,3);
                    position.z = unimap.nodes[p].tf(2,3) + 1.0;
                    viewer2->addText3D(id.toStdString(), position, 0.5, 0.0, 1.0, 0.0, text_id.toStdString());
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

                        viewer2->addLine(pt0, pt1, 1.0, 0.0, 0.0, id0.toStdString());
                        viewer2->addLine(pt1, pt2, 0.0, 1.0, 0.0, id1.toStdString());

                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id0.toStdString());
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id1.toStdString());
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id0.toStdString());
                        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id1.toStdString());

                        last_plot_edges.push_back(id0);
                        last_plot_edges.push_back(id1);
                    }
                }
            }
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

    if(ui->tabAnnotation->currentIndex() == TAB_MAP)
    {
    }
    else if(ui->tabAnnotation->currentIndex() == TAB_TOPO)
    {
        // draw selection
        if(pick.pre_node != "")
        {
            NODE *node = unimap.get_node_by_id(pick.pre_node);
            if(node != NULL)
            {
                pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.1, node->tf, 1.0, 0.0, 0.0);
                viewer2->addPolygonMesh(donut, "sel_pre");
            }
        }

        if(pick.cur_node != "")
        {
            NODE *node = unimap.get_node_by_id(pick.cur_node);
            if(node != NULL)
            {
                pcl::PolygonMesh donut = make_donut(config.ROBOT_RADIUS, 0.1, node->tf, 0.0, 1.0, 0.0);
                viewer2->addPolygonMesh(donut, "sel_cur");
            }
        }

        // draw pose
        viewer2->addCoordinateSystem(1.0, "O_pick");
        if(ui->cb_NodeType->currentText() == "ROUTE")
        {
            viewer2->addSphere(pcl::PointXYZ(0, 0, 0), 0.15, 1.0, 0.0, 1.0, "pick_body");
            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
            viewer2->updateShapePose("pick_body", Eigen::Affine3f(ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]).cast<float>()));
        }
        else if(ui->cb_NodeType->currentText() == "GOAL")
        {
            viewer2->addCube(config.ROBOT_SIZE_X[0], config.ROBOT_SIZE_X[1],
                             config.ROBOT_SIZE_Y[0], config.ROBOT_SIZE_Y[1],
                             config.ROBOT_SIZE_Z[0], config.ROBOT_SIZE_Z[1], 1.0, 0.0, 1.0, "pick_body");
            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
            viewer2->updateShapePose("pick_body", Eigen::Affine3f(ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]).cast<float>()));
        }
        else if(ui->cb_NodeType->currentText() == "CROSS")
        {
            viewer2->addSphere(pcl::PointXYZ(0, 0, 0), 0.15, 1.0, 0.0, 1.0, "pick_body");
            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
            viewer2->updateShapePose("pick_body", Eigen::Affine3f(ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]).cast<float>()));
        }
        else if(ui->cb_NodeType->currentText() == "SIGN")
        {
            viewer2->addCube(-0.15, 0.15, -0.15, 0.15, 0.0, 2.0, 1.0, 0.0, 1.0, "pick_body");
            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
            viewer2->updateShapePose("pick_body", Eigen::Affine3f(ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]).cast<float>()));
        }
        else if(ui->cb_NodeType->currentText() == "STAIR")
        {
            Eigen::Quaternionf rot(Eigen::AngleAxisf(-M_PI/4, Eigen::Vector3f::UnitY()));
            viewer2->addCube(Eigen::Vector3f(0,0,0), rot, 0.5, 0.25, 0.05, "pick_body");
            viewer2->updateShapePose("pick_body", Eigen::Affine3f(ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]).cast<float>()));
            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "pick_body");
            viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "pick_body");
        }

        viewer2->updateCoordinateSystemPose("O_pick", Eigen::Affine3f(ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]).cast<float>()));
    }

    // rendering
    ui->qvtkWidget2->renderWindow()->Render();
}
