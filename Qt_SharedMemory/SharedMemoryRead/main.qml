import QtQuick 2.9
import QtQuick.Window 2.2

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("Hello World")

    Text {
        anchors.centerIn: parent
        text: "shared memory read"
        font {
            pixelSize: 50
        }
    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            HMI_CTRL.readSharedMemory()
        }
    }
}
