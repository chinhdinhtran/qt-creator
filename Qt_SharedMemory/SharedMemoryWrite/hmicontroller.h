#ifndef HMICONTROLLER_H
#define HMICONTROLLER_H

#include <QSharedMemory>
#include <QObject>
#include <QDBusMessage>
#include <QDBusConnection>
#include <QDBusPendingReply>

struct SHARE_INFO {
    int id;
    char message[1000];
};

class HMIController : public QObject
{
    Q_OBJECT

public:
    explicit HMIController(QObject *parent = nullptr);

    Q_INVOKABLE bool writeSharedMemory();
    Q_INVOKABLE void broadcastDbusSignal();

private:
    QSharedMemory mSharedMemWrite;
    int m_testCount;
};

#endif // HMICONTROLLER_H
