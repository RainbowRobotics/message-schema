#include "fms.h"

FMS::FMS(QObject *parent)
    : QObject{parent}
{

}

FMS::~FMS()
{

}

void FMS::init()
{
    // set id
    QString _id;
    _id.sprintf("R_%lld", (long long)(get_time0()*1000));
    id = _id;

    printf("[FMS] ID: %s\n", _id.toLocal8Bit().data());
}
