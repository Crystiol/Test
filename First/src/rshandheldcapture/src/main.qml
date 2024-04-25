import QtQuick 2.0
import QtQuick.Window 2.12
import QtQuick.Controls 1.2
import QtQuick.Layouts 1.3
import jm.qt.RSSceneViewer 1.0

Window {
    objectName : "rootObject";
    width: 640
    height: 480
    visible: true
    visibility : Window.Maximized
    title: qsTr("Hello World")

    ColumnLayout
    {
        anchors.fill: parent;

        RSSceneViewer {
            id: viewer;
            objectName : "qmlviewer";
            Layout.fillWidth: true;
            Layout.fillHeight: true;
        }

        RowLayout{
            Layout.leftMargin: 4;
            Layout.bottomMargin: 4;
            Layout.rightMargin: 4;
            Layout.fillWidth: true;
            height: 32;

            Button {
                    id: btnAddCloud;
                    text: "AddCloud";
                    Layout.fillWidth: true
                    Layout.minimumHeight: 30

                    onClicked:{
                        var vectorList = [
                                        [0.2, 0.0, 0.0],
                                        [0.5, 0.0, 0.0],
                                        [1.0, 0.5, 0.0],
                                        [1.5, 1.0, 0.0],
                                        [2.5, 1.5, 0.0],
                                        [1.5, 2.5, 0.0],
                                        [0.5, 1.0, 0.0]
                                    ];
                        viewer.addPointCloud(vectorList);
                    }
                }

            Button {
                    id: btnAddTrack;
                    text: "AddTrack";
                    Layout.fillWidth: true;
                    Layout.minimumHeight: 30;

                    onClicked:{
                        //var p = Qt.vector3d(1,0,0);
                        //viewer.processVector([p]);
                        //viewer.addTrackPathPoint([p]);

                        var vectorList = [
                                        [0.2, 0.0, 0.0],
                                        [0.5, 0.0, 0.0],
                                        [1.0, 0.5, 0.0],
                                        [1.5, 1.0, 0.0],
                                        [2.5, 1.5, 0.0],
                                        [1.5, 2.5, 0.0],
                                        [0.5, 1.0, 0.0]
                                    ];
                        viewer.addTrackPathPoints(vectorList);
                    }
                }

            Button {
                    id: btnChange;
                    text: "Change";
                    Layout.fillWidth: true
                    Layout.minimumHeight: 30

                    onClicked:{
                        var color = [1.0,1.0,1.0];
                        viewer.setBackGround(color);
                    }
                }

            Button {
                    id: btnCamera;
                    text: "Camera";
                    Layout.fillWidth: true
                    Layout.minimumHeight: 30

                    onClicked:{

                    }
                }

            Button {
                    id: btnClear;
                    text: "Clear";
                    Layout.fillWidth: true
                    Layout.minimumHeight: 30

                    onClicked:{
                        viewer.clear();
                    }
                }
        }
    }
}
