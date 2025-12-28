//
// Created by jy on 25. 12. 27..
//

#ifndef SLAMNAV2_UI_TYPES_H
#define SLAMNAV2_UI_TYPES_H

// vtk
#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkDataObjectToTable.h>
#include <vtkElevationFilter.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkQtTableView.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>

struct CPU_USAGE
{
    long long user = 0;
    long long nice = 0;
    long long system = 0;
    long long idle = 0;
    long long iowait = 0;
    long long irq = 0;
    long long softirq = 0;

    CPU_USAGE()
    {
        user = 0;
        nice = 0;
        system = 0;
        idle = 0;
        iowait = 0;
        irq = 0;
        softirq = 0;
    }

    CPU_USAGE(const CPU_USAGE& p)
    {
        user = p.user;
        nice = p.nice;
        system = p.system;
        idle = p.idle;
        iowait = p.iowait;
        irq = p.irq;
        softirq = p.softirq;
    }

    CPU_USAGE& operator=(const CPU_USAGE& p)
    {
        user = p.user;
        nice = p.nice;
        system = p.system;
        idle = p.idle;
        iowait = p.iowait;
        irq = p.irq;
        softirq = p.softirq;
        return *this;
    }
};
#endif //SLAMNAV2_UI_TYPES_H