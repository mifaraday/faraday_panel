#include "faradaypanel.h"
#include "ui_faradaypanel.h"

namespace faraday_panel
{

FaradayPanel::FaradayPanel(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::FaradayPanel),
    m_step_(0.16),
    m_speed_(50.0),
    m_busInterface_(1),
    canControl_(false),
    spindleControl_(false),
    spindleVelocity_(0)

{
    ui->setupUi(this);

    ui->pm_can->setChecked(false);

    m_timer_=new QTimer(this);
    connect(m_timer_,SIGNAL(timeout()), this, SLOT(sendCommand()));

    initROS();

    // set the timer periods 10ms
    m_timer_->start(10);
}

FaradayPanel::~FaradayPanel()
{
    delete ui;
}

void FaradayPanel::initROS()
{
    spindle_publisher_ = nh_.advertise<faraday_panel::Spindle>("pcan_cmd", 1000);
    cartesian_publisher_ = nh_.advertise<sensor_msgs::JointState>("send_goal", 1);
    subSpindleState_ = nh_.subscribe("spindle_state", 1000, &FaradayPanel::spindleStateCallback,this);
    subJointState_ = nh_.subscribe("joint_states", 1000, &FaradayPanel::jointStateCallback,this);

}

int FaradayPanel::pointCompare(void)
{
  int ret = 0;
  for(int i=0;i<6;i++)
  {
    if(fabs(jointPosition_[i]-lastJointPosition_[i])>=0.000001)
    {
       ret = 1;
       break;
    }
  }
  return ret;
}

void FaradayPanel::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    lastJointPosition_[0] = msg->position[0];
    lastJointPosition_[1] = msg->position[1];
    lastJointPosition_[2] = msg->position[2];
    lastJointPosition_[3] = msg->position[3];
    lastJointPosition_[4] = msg->position[4];
    lastJointPosition_[5] = msg->position[5];

    if(pointCompare())
    {
        jointPosition_[0] = lastJointPosition_[0];
        jointPosition_[1] = lastJointPosition_[1];
        jointPosition_[2] = lastJointPosition_[2];
        jointPosition_[3] = lastJointPosition_[3];
        jointPosition_[4] = lastJointPosition_[4];
        jointPosition_[5] = lastJointPosition_[5];

        ui->lb_joint1->setText(QString::number((jointPosition_[0]),'f',6 ));
        ui->lb_joint2->setText(QString::number((jointPosition_[1]),'f',6 ));
        ui->lb_joint3->setText(QString::number((jointPosition_[2]),'f',6 ));
        ui->lb_joint4->setText(QString::number((jointPosition_[3]),'f',6 ));
        ui->lb_joint5->setText(QString::number((jointPosition_[4]),'f',6 ));
        ui->lb_joint6->setText(QString::number((jointPosition_[5]),'f',6 ));
    }

}

void FaradayPanel::spindleStateCallback(const faraday_panel::Spindle::ConstPtr& msg)
{
    float spindleSpeed=msg->spindleSpeedFB;

    ui->lb_spindle->setText(QString::number((spindleSpeed),'f',6 ));
}


void FaradayPanel::on_pm_can_clicked()
{
    if(!canControl_)
    {
        ui->pm_can->setChecked(true);

        m_busInterface_=1;

        canControl_=true;

        spindleControl_=false;

        spindleVelocity_=0.0;

        pcanMsg_.isCanLink=canControl_;
        pcanMsg_.isSpindleStart=spindleControl_;
        pcanMsg_.spindleSpeed=spindleVelocity_;

        spindle_publisher_.publish(pcanMsg_);
    }
    else
    {
        ui->pm_can->setChecked(false);

        m_busInterface_=0;

        canControl_=false;

        spindleControl_=false;

        spindleVelocity_=0.0;

        pcanMsg_.isCanLink=canControl_;
        pcanMsg_.isSpindleStart=spindleControl_;
        pcanMsg_.spindleSpeed=spindleVelocity_;

        spindle_publisher_.publish(pcanMsg_);
    }
}


void FaradayPanel::on_pbn_spindleLeft_pressed()
{
    if(m_busInterface_ == 0) m_step_=0.16;
    else m_step_=0.035;

    spindleVelocity_= spindleVelocity_ - m_step_ * m_speed_ / 100 < 500 ?
                      spindleVelocity_ :
                      spindleVelocity_ - m_step_ * m_speed_ / 100;

    if(spindleVelocity_ < 0) spindleVelocity_ = 0;

    ui->lb_spindle->setText(QString::number(spindleVelocity_, 'f', 6));

    pcanMsg_.spindleSpeed=spindleVelocity_;

    spindle_publisher_.publish(pcanMsg_);
}

void FaradayPanel::on_pbn_spindleRight_pressed()
{
    if(m_busInterface_ == 0) m_step_=0.16;
    else m_step_=0.035;

    spindleVelocity_= spindleVelocity_ + m_step_ * m_speed_ / 100 < 50 ?
                      spindleVelocity_ :
                      spindleVelocity_ + m_step_ * m_speed_ / 100;

    if(spindleVelocity_ > 8500) spindleVelocity_ = 8500;

    ui->lb_spindle->setText(QString::number(spindleVelocity_, 'f', 6));

    pcanMsg_.spindleSpeed=spindleVelocity_;

    spindle_publisher_.publish(pcanMsg_);

}

void FaradayPanel::on_pbn_spindleStart_clicked()
{
    spindleControl_=true;

    pcanMsg_.isCanLink=spindleControl_;

    spindle_publisher_.publish(pcanMsg_);

}

void FaradayPanel::on_pbn_spindleStop_clicked()
{
    spindleControl_=false;

    pcanMsg_.isCanLink=spindleControl_;

    spindle_publisher_.publish(pcanMsg_);
}

void FaradayPanel::on_pbn_spindleSend_clicked()
{

    pcanMsg_.spindleSpeed=spindleVelocity_;

    spindle_publisher_.publish(pcanMsg_);
}

void FaradayPanel::on_le_spindle_editingFinished()
{
    QString temp_string = ui->le_spindle->text();

    float lin = temp_string.toFloat();

    spindleVelocity_ = lin;
}

void FaradayPanel::on_pbn_xLeft_pressed()
{

}

void FaradayPanel::on_pbn_xRight_pressed()
{

}

void FaradayPanel::on_pbn_yLeft_pressed()
{

}

void FaradayPanel::on_pbn_yRight_pressed()
{

}

void FaradayPanel::on_pbn_zLeft_pressed()
{

}

void FaradayPanel::on_pbn_zRight_pressed()
{

}

void FaradayPanel::on_pbn_rollLeft_pressed()
{

}

void FaradayPanel::on_pbn_rollRight_pressed()
{

}

void FaradayPanel::on_pbn_pitchLeft_pressed()
{

}

void FaradayPanel::on_pbn_pitchRight_pressed()
{

}

void FaradayPanel::on_le_X_editingFinished()
{

}

void FaradayPanel::on_le_Y_editingFinished()
{

}

void FaradayPanel::on_le_Z_editingFinished()
{

}

void FaradayPanel::on_le_roll_editingFinished()
{

}

void FaradayPanel::on_le_pitch_editingFinished()
{

}

void FaradayPanel::on_pbn_zero_clicked()
{

}

void FaradayPanel::on_pbn_classicTraj1_clicked()
{

}

void FaradayPanel::on_pbn_classicTraj2_clicked()
{

}

void FaradayPanel::on_pbn_classicTraj3_clicked()
{

}

void FaradayPanel::on_pbn_sendGoal_clicked()
{

}

void FaradayPanel::on_pbn_eStop_clicked()
{

}

void FaradayPanel::sendCommand()
{
    if( ros::ok()&&(spindle_publisher_))
    {
        ROS_WARN("Send");
        if(m_busInterface_ == 0)
        {
            spindle_publisher_.publish(pcanMsg_);
        }
    }
}

} // end of namespace faraday_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(faraday_panel::FaradayPanel,rviz::Panel)



