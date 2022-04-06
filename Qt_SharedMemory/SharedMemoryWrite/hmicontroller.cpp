#include "hmicontroller.h"
#include "loghelper.h"

HMIController::HMIController(QObject *parent)
    : QObject(parent)
    , m_testCount(0)
{
    QString ss = "AAAAAAAAAAAAAAAAAAAAAAAA";
    LOG_INFO << sizeof (ss);

    mSharedMemWrite.setKey("TestShareMemKey");
    if(mSharedMemWrite.attach())
    {
        LOG_INFO << "attach shared mem";
    }
    if(mSharedMemWrite.create(sizeof(SHARE_INFO)))
    {
        LOG_INFO << "create shared mem";
    }
}

bool HMIController::writeSharedMemory()
{
    LOG_INFO << m_testCount;

    if(!mSharedMemWrite.isAttached())
    {
        LOG_INFO << "Attached (Failure)";
        return false;
    }
    if(!mSharedMemWrite.lock())
    {
        LOG_INFO << "Lock (Failure)";
        return false;
    }

    char* buffer = (char*)mSharedMemWrite.data();
    if(buffer == nullptr)
    {
        mSharedMemWrite.unlock();
        LOG_INFO << "Data (Failure)";
        return false;
    }

    // create test data
    SHARE_INFO info;
    info.id = m_testCount;
    strcpy(info.message, "Test message");

    // copy data to shared mem
    memcpy(buffer, &info, sizeof(SHARE_INFO));

    // unlock
    mSharedMemWrite.unlock();

    // increase test count
    m_testCount++;
    broadcastDbusSignal();
    return true;
}

void HMIController::broadcastDbusSignal()
{
    LOG_INFO;

    QDBusMessage msg = QDBusMessage::createSignal(
                "/vn/com/chingio/appInterface",
                "vn.com.chingio.appInterface",
                "notifySignal"
                );
    msg << 1 << "9h hello world - write data is done";
    QDBusConnection::sessionBus().send(msg);
    LOG_INFO <<"send done";
}
