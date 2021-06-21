import QtQuick 2.12
import QtQuick.Controls 2.12

Rectangle
{
    id: container
    property alias gripperPos: gripperPos_txt.text
    property alias armName: name_txt.text
    property bool enableArm: true
    property bool allArms: false
    property int margin: 10

    implicitWidth: Math.max(name_txt.width + 2*margin,
                            gripper_btn.width + release_btn.width + wrist_btn.width + !allArms*gripperPos_rect.width + calibrate_btn.width + 6*margin)
    implicitHeight: name_txt.height + gripper_btn.height + 3*margin
    border.color: "black"
    radius: margin
    enabled: !calibrate_btn.buttonChecked && enableArm

    Text
    {
        id: name_txt
        anchors.margins: margin
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        font.bold: true
    }

    Row {
        anchors.top: name_txt.bottom
        anchors.topMargin: margin
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: margin

        Button
        {
            property bool buttonChecked: false
            id: calibrate_btn
            text: buttonChecked ? qsTr("Calibrating") : qsTr("Calibrate")
            onClicked:
            {
                buttonChecked = !buttonChecked
                if(buttonChecked)
                    delayTimer.start();

                if(allArms)
                    ArmsController.calibrateAllGrippers()
                else
                    ArmsController.calibrateGripper(armName)
            }

            Timer {
                id: delayTimer
                interval: 5000
                repeat: false
                onTriggered: calibrate_btn.buttonChecked = !calibrate_btn.buttonChecked
            }
        }

        Button
        {
            id: release_btn
            text: qsTr("Release")
            onClicked:
            {
                if(allArms)
                    ArmsController.releaseAllGrippers()
                else
                    ArmsController.releaseGripper(armName)
            }
        }

        Button
        {
            property bool buttonChecked: false
            id: wrist_btn
            text: buttonChecked ? qsTr("Return") : qsTr("Twist")
            onClicked:
            {
                buttonChecked = !buttonChecked
                if(allArms)
                    ArmsController.twistAllWrists(buttonChecked)
                else
                    ArmsController.twistWrist(armName, buttonChecked)
            }
        }

        Button
        {
            property bool buttonChecked: false
            id: gripper_btn
            text: buttonChecked ? qsTr("Open") : qsTr("Close")
            onClicked:
            {
                if(buttonChecked)
                {
                    if(allArms)
                        ArmsController.openAllGrippers()
                    else
                        ArmsController.openGripper(armName)
                }
                else
                {
                    if(allArms)
                        ArmsController.closeAllGrippers()
                    else
                        ArmsController.closeGripper(armName)
                }
                buttonChecked = !buttonChecked
            }

        }

        Rectangle {
            visible: !allArms
            id: gripperPos_rect
            implicitWidth: childrenRect.width
            implicitHeight: childrenRect.height
            anchors.verticalCenter: gripper_btn.verticalCenter
            Text {
                id: prefix_txt
                text: qsTr("Gripper Pos:")
            }
            Text {
                id: gripperPos_txt
                color: "red"
                anchors.left: prefix_txt.right
                anchors.leftMargin: 3
                width: 30
            }
        }
    }
}
