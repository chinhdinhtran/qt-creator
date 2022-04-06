#ifndef LOGHELPER_H
#define LOGHELPER_H

#include <QDebug>
#include <unistd.h>
//#include <sys/syscall.h>

#define LOG_INFO qDebug().nospace() << Q_FUNC_INFO << " "

#endif // LOGHELPER_H
