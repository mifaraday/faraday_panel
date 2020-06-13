#ifndef FARADAYPANEL_H
#define FARADAYPANEL_H

//#ifndef Q_MOC_RUN
#include <QWidget>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#include <faraday_panel/Spindle.h>
#include <std_msgs/Float32MultiArray.h>
#include <QTimer>
//#endif

namespace Ui {
class FaradayPanel;
}

namespace faraday_panel
{

class FaradayPanel : public rviz::Panel
{
    Q_OBJECT

public:
    explicit FaradayPanel(QWidget *parent = 0);
    ~FaradayPanel();

public:
    void initROS();

private Q_SLOTS:
    void on_pm_can_clicked();

    void on_pbn_spindleLeft_pressed();

    void on_pbn_spindleRight_pressed();

    void on_pbn_spindleStart_clicked();

    void on_pbn_spindleStop_clicked();

    void on_pbn_spindleSend_clicked();

    void on_le_spindle_editingFinished();

    void on_pbn_xLeft_pressed();

    void on_pbn_xRight_pressed();

    void on_pbn_yLeft_pressed();

    void on_pbn_yRight_pressed();

    void on_pbn_zLeft_pressed();

    void on_pbn_zRight_pressed();

    void on_pbn_rollLeft_pressed();

    void on_pbn_rollRight_pressed();

    void on_pbn_pitchLeft_pressed();

    void on_pbn_pitchRight_pressed();

    void on_le_X_editingFinished();

    void on_le_Y_editingFinished();

    void on_le_Z_editingFinished();

    void on_le_roll_editingFinished();

    void on_le_pitch_editingFinished();

    void on_pbn_zero_clicked();

    void on_pbn_classicTraj1_clicked();

    void on_pbn_classicTraj2_clicked();

    void on_pbn_classicTraj3_clicked();

    void on_pbn_sendGoal_clicked();

    void on_pbn_eStop_clicked();

protected Q_SLOTS:
    void sendCommand();

protected:
    QTimer* m_timer_;

    // The ROS publisher/subscriber

    // send can and spindle message
    ros::Publisher spindle_publisher_;

    ros::Publisher cartesian_publisher_;

    // send cartesian and joint message
    sensor_msgs::JointState jointMsg_;

    faraday_panel::Spindle pcanMsg_;

    ros::Subscriber subJointState_;

    ros::Subscriber subSpindleState_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    int pointCompare(void);
    void spindleStateCallback(const faraday_panel::Spindle::ConstPtr& msg);

    // The ROS node handle.
    ros::NodeHandle nh_;

    bool canControl_;
    bool spindleControl_;
    float spindleVelocity_;
    float jointPosition_[7];
    float lastSpindleVlocity_;
    float lastJointPosition_[7];

private:
    Ui::FaradayPanel *ui;
    float m_step_;
    float m_speed_;

    int m_busInterface_;
};

} // end of namespace faraday_panel

#endif // FARADAYPANEL_H
