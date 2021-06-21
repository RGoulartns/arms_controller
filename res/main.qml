import QtQuick 2.15
import QtQuick.Window 2.15
import "qml"

import QtQuick.Controls 2.12

Window {
    id: mainWindow
    width: 640
    height: 480
    visible: true
    title: qsTr("Arms Controller")

    Column {
        id: itemsColumn
        anchors.centerIn: parent
        spacing: 25
        Repeater {
            model: ArmsController.armsList
            delegate: ArmControl{
                armName: model.modelData
                gripperPos: ArmsController.grippersPos[model.modelData]
                enableArm: allArmsComponent.enabled
            }
        }
        ArmControl {
            id: allArmsComponent
            anchors.horizontalCenter: parent.horizontalCenter
            armName: "All"
            allArms: true
        }
    }
    /*
    ListView {
        anchors.fill: parent
        clip: true
        spacing: 25
        orientation: ListView.Horizontal
        model: ArmsController.armsList //g1,g2,g3 ...
        delegate: ArmControl{
            armName: model.modelData
        }
        footer: ArmControl {
            armName: "All Grippers"
            allArms: true
        }
        footerPositioning: ListView.OverlayFooter
    }
    */
    /*
    Component.onCompleted: {
        var component = Qt.createComponent("qml/ArmControl.qml");
        for (var i=0; i<10; i++) {
            var object = component.createObject(mainWindow);
            object.armName = "qwer"
            //object.anchors.verticalCenter = object.parent.verticalCenter
            //object.anchors.horizontalCenter = object.parent.horizontalCenter
            object.x = (object.width + 50)*i;
        }
    }
    */
}
