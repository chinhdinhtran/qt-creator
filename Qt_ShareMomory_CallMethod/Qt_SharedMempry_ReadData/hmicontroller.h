#ifndef HMICONTROLLER_H
#define HMICONTROLLER_H

#include <QObject>
#include <QSharedMemory>
#include <QDBusMessage>
#include <QDBusConnection>
#include <QDBusPendingReply>
#include "appmanager_adaptor.h"

struct SHARE_DATA {
    int id;
    char name[1000];
};

class HMIController : public QObject
{
    Q_OBJECT
public:
    explicit HMIController(QObject *parent = nullptr);
    ~HMIController();

private:
    AppInterfaceAdaptor *m_adaptor = nullptr;
    QSharedMemory mSharedMemRead;
    void readShareMemory();

    // method call
    Q_INVOKABLE QString methodDbusCall(int param1, const QString &param2);
};

#endif // HMICONTROLLER_H
