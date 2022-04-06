#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "hmicontroller.h"

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    HMIController hmiController;
    engine.rootContext()->setContextProperty("HMI_CTRL", &hmiController);

    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
