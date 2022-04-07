#ifndef LOGGER_H
#define LOGGER_H


#include <QDebug>
#include <unistd.h>

#define LOG_INFO qDebug().nospace() << Q_FUNC_INFO << " "


#endif // LOGGER_H
