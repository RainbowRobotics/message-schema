#ifndef QR_SENSOR_H
#define QR_SENSOR_H

#include <QObject>

class QR_SENSOR : public QObject
{
    Q_OBJECT
public:
    explicit QR_SENSOR(QObject *parent = nullptr);

Q_SIGNALS:

};

#endif // QR_SENSOR_H
