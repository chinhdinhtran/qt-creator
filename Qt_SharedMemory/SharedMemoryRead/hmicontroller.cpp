#include "hmicontroller.h"
#include "loghelper.h"

HMIController::HMIController(QObject *parent)
    : QObject(parent)
{
    LOG_INFO;

    QDBusConnection::sessionBus().connect(
                "",
                "/vn/com/chingio/appInterface",
                "vn.com.chingio.appInterface",
                "notifySignal",
                this, SLOT(onReceivedSignal(int , QString)));

    mSharedMemRead.setKey("TestShareMemKey");

}

void HMIController::readSharedMemory()
{
    LOG_INFO;

    int count = 0;
    bool isAttachSuccess = mSharedMemRead.attach();
    while(!isAttachSuccess && (count++ < 2))
    {
        usleep(100000);
        isAttachSuccess = mSharedMemRead.attach();
    }

    if(isAttachSuccess)
    {
        if(!mSharedMemRead.lock())
        {
            LOG_INFO << "Cannot lock";
        }
        else
        {
            SHARE_INFO info = {};
            memcpy(&info, mSharedMemRead.constData(), sizeof(info));

            // unlock
            mSharedMemRead.unlock();

            // print shared info
            LOG_INFO << "id: " << info.id;
            LOG_INFO << "message: " << QString::fromUtf8(info.message);
        }

        // detach
        mSharedMemRead.detach();
    }
    else
    {
        LOG_INFO << "Cannot attach: " << mSharedMemRead.errorString();
    }
}

void HMIController::onReceivedSignal(int param1, QString param2)
{
    LOG_INFO << param1 << " " << param2;
    LOG_INFO << "Read share memory";
    readSharedMemory();
}
