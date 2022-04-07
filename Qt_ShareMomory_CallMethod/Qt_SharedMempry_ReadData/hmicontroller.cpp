#include "hmicontroller.h"
#include "Logger.h"

HMIController::HMIController(QObject *parent) : QObject(parent)
{
    LOG_INFO;
    mSharedMemRead.setKey("9h_key");

    // ---------- config dbus for method call
    m_adaptor = new AppInterfaceAdaptor(this);
    QDBusConnection bus = QDBusConnection::sessionBus();
    if(!bus.isConnected())
    {
        LOG_INFO << "Can not connect to dbus";
    }
    if(!bus.registerService("vn.com.chingio"))
    {
        LOG_INFO << "Can not register service";
    }
    if(!bus.registerObject("/vn/com/chingio/appInterface", this))
    {
        LOG_INFO << "Can not register object path";
    }
}

HMIController::~HMIController()
{
    LOG_INFO;
    if(m_adaptor != nullptr)
    {
        delete m_adaptor;
        m_adaptor = nullptr;
    }
}

void HMIController::readShareMemory()
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
            SHARE_DATA data = {};
            memcpy(&data, mSharedMemRead.constData(), sizeof(data));

            // unlock
            mSharedMemRead.unlock();

            // print shared info
            LOG_INFO << "id: " << data.id;
            LOG_INFO << "name: " << QString::fromUtf8(data.name);
        }

        // detach
        mSharedMemRead.detach();
    }
    else
    {
        LOG_INFO << "Cannot attach: " << mSharedMemRead.errorString();
    }
}

QString HMIController::methodDbusCall(int param1, const QString &param2)
{
    LOG_INFO << param1 << " " << param2;
    readShareMemory();
    return "9h call method successful";
}
