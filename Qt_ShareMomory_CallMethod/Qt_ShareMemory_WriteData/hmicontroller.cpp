#include "hmicontroller.h"
#include "logger.h"


HMIController::HMIController(QObject *parent) : QObject(parent), mCount{0}
{
    mShareMemWrite.setKey("9h_key");
    if(mShareMemWrite.attach()){
        LOG_INFO << "Attach shared memory is done";
    }
    if(mShareMemWrite.create(sizeof(SHARE_DATA))){
        LOG_INFO << "Create shared memory is done";
    }
}

bool HMIController::writeShareMemory()
{
    LOG_INFO << "mCount : "<< mCount;
    if(!mShareMemWrite.isAttached()){
        LOG_INFO << "Attached shared is fail";
        return false;
    }
    if(!mShareMemWrite.lock()){
        LOG_INFO << "Lock shared is fail";
        return false;
    }

    char *buffer = (char*)mShareMemWrite.data();
    if(buffer == nullptr){
        mShareMemWrite.unlock();
        LOG_INFO << "Data is null";
        return false;
    }

    //create data
    SHARE_DATA data;
    data.id = mCount;
    strcpy(data.name, "Chin gio");
    memcpy(buffer, &data, sizeof(SHARE_DATA));

    //unclock
    mShareMemWrite.unlock();

    //increase mCount when qml call write shared memory
    mCount++;

    //call mothod for riceived app automatic is actived
    callDbusMethod();
    return true;

}
void HMIController::callDbusMethod()
{
    LOG_INFO;

    // 1. create a dbus method call
    QDBusMessage msg = QDBusMessage::createMethodCall("vn.com.chingio",                   // service name
                                                      "/vn/com/chingio/appInterface",     // object path
                                                      "vn.com.chingio.appInterface",      // service interface
                                                      "methodDbusCall");                // method name

    // 2. append input arguments
    msg << 1 << "9h call method request read data";

    // 3. call method (using call or asyncCall)
    QDBusMessage response = QDBusConnection::sessionBus().call(msg);

    // 4. check return value
    if(!response.errorMessage().isEmpty())
    {
        LOG_INFO << "response error: " << response.errorMessage();
    }
    else
    {
        LOG_INFO << "call method successfully " << response.arguments().at(0).toString();
    }
}
