#ifndef HMICONTROLLER_H
#define HMICONTROLLER_H

#include <QObject>
#include <QSharedMemory>
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

    Q_INVOKABLE void readSharedMemory();
    Q_INVOKABLE void onReceivedSignal(int param1, QString param2);

private:
    QSharedMemory mSharedMemRead;
};

#endif // HMICONTROLLER_H
