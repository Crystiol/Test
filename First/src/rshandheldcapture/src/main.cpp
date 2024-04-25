#include <QQmlApplicationEngine>

#include "RSApplication.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc,char **argv)
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    RSApplication app(argc, argv, "view");

    qmlRegisterType<RSSceneViewer>("jm.qt.RSSceneViewer", 1, 0, "RSSceneViewer");

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreated,
        &app,
        [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        },
        Qt::QueuedConnection);
    engine.load(url);

    QObject * root = NULL;
    QList<QObject*> rootObjects = engine.rootObjects();
    int count = rootObjects.size();
    for(int i = 0; i < count; i++)
    {
        if(rootObjects.at(i)->objectName() == "rootObject")
        {
            root = rootObjects.at(i);
            break;
        }
    }

    if(root)
    {
        QObject * viewer = root->findChild<QObject*>("qmlviewer");
        if(viewer)
        {
            QVariantList colorList;
            colorList << 0.5 << 0.6 << 0.7;
            QMetaObject::invokeMethod(viewer, "setBackGround", Q_ARG(QVariantList, colorList));

            app.SetView(qobject_cast<RSSceneViewer*>(viewer));
        }
    }

    return app.exec();
}
