#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // set plot window
    setup_vtk();
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

    renderer->GetActiveCamera()->SetParallelProjection(1);
    viewer->resetCamera();
    ui->qvtkWidget->setMouseTracking(true);

    // init drawing
    viewer->addCoordinateSystem(1.0, "O_global");
}
