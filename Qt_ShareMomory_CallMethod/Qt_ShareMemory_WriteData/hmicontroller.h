#ifndef HMICONTROLLER_H
#define HMICONTROLLER_H

#include <QObject>
#include <QSharedMemory>
#include <QDBusMessage>
#include <QDBusConnection>
#include <QDBusPendingReply>




struct SHARE_DATA{
    int id;
    char name[1000];
};

class HMIController : public QObject
{
    Q_OBJECT
public:
    explicit HMIController(QObject *parent = nullptr);

public slots:
    bool writeShareMemory();
    void callDbusMethod();

private:
    QSharedMemory mShareMemWrite;
    int mCount;
};

#endif // HMICONTROLLER_H
