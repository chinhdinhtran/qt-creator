import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.12

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("Hello World")
    Button{
        anchors.centerIn: parent
        height: 300
        width: 300
        text: "Write shared memory."
        onClicked: {
            HMI_CTRL.writeShareMemory();
            console.log("HMI_CTRL.writeShareMemory() is called from qml")
        }
    }
}
