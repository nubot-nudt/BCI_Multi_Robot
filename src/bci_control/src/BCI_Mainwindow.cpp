#include "BCI_Mainwindow.h"
#include "ui_BCI_Mainwindow.h"

MainWindow::MainWindow(nubot::interface_Info & robot2BCI, nubot::BCIcontrol_Info & BCI2robot, nubot::BCI_signal &BCIsignal, QWidget *parent) :
    QMainWindow (parent),
    ui(new Ui::MainWindow)
{
//    setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
//    setAttribute(Qt::WA_TranslucentBackground, true);
    robot2BCI_info_= & robot2BCI;
    BCI2robot_info_= & BCI2robot;
    BCI_signal_info_= & BCIsignal;

    scene_=new QGraphicsScene();

    QTimer *timer=new QTimer(this);
    tcpSocket_=new QTcpSocket(this);

    QByteArray *buffer = new QByteArray();
    qint32 *s = new qint32(0);
    buffers.insert(tcpSocket_, buffer);
    sizes.insert(tcpSocket_, s);

    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpdate()));
    connect(tcpSocket_, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError_(QAbstractSocket::SocketError)));

    ui->setupUi(this);
    this->setFixedSize(this->width(),this->height());

    ui->display->setScene(scene_);

    //set default ip
    QRegExp rx("((2[0-4]//d|25[0-5]|[01]?//d//d?)//.){3}(2[0-4]//d|25[0-5]|[01]?//d//d?)");
    QRegExpValidator v(rx, nullptr);

    ui->IP_in->setValidator(&v);
    ui->IP_in->setInputMask("000.00.0.0;0");
    ui->IP_in->setText("172.16.1.2");

    timer->start(5);

    //load field pic
    field_img_.load(":/field.png");
    field_=scene_->addPixmap(field_img_);
    scene_->setSceneRect(0,0,ui->display->width()-2,ui->display->height()-2);

    //load robot pic
    for(unsigned i=0;i<TEAM_NUM;i++)
    {
        robot_img_[i].load(":/NUM"+QString::number(i+1)+".png");
        robot_[i]=scene_->addPixmap(robot_img_[i]);
        robot_[i]->setPos(40+i*55,665);
    }

    //load opponent pic
    for(unsigned i=0; i<OPP_TEAM_NUM; i++)
    {
        opponent_img_[i].load(":/OPP"+QString::number(i+1)+".png");
        opponent_[i]=scene_->addPixmap(opponent_img_[i]);
        opponent_[i]->setPos(900,900);
    }

    //load velocity pic
    for(unsigned i=0; i<OPP_TEAM_NUM; i++)
    {
        velocity_img_.load(":/velocity.png");
        velocity_[i]=scene_->addPixmap(velocity_img_);
        velocity_[i]->setPos(900,900);
    }

    //load ball pic
    ball_img_.load(":/ball.png");
    ball_=scene_->addPixmap(ball_img_);
    ball_->setPos(350,670);

    //initialization the buttons
    select_buttons_<<ui->selection1<<ui->selection2<<ui->selection3<<ui->selection4<<ui->selection5<<ui->selection6;
    for(int i=0;i<select_buttons_.size();i++)
    {
        select_buttons_[i]->setFlat(true);
        select_buttons_[i]->setIconSize(QSize(100,100));
    }

    //initialization the icons
    strategy_icons_<<QIcon(":/select_attack.png")<<QIcon(":/select_defend.png")<<QIcon(":/select_robot.png")<<QIcon(":/select_stop.png");
    attack_icons_<<QIcon(":/select_balance.png")<<QIcon(":/select_radical.png")<<QIcon(":/select_conservative.png")<<QIcon(":/select_return.png");
    defend_icons_<<QIcon(":/select_regional.png")<<QIcon(":/select_man2man.png")<<QIcon(":/select_focus.png")<<QIcon(":/select_return.png");
    nubot_icons_<<QIcon(":/select_one.png")<<QIcon(":/select_two.png")<<QIcon(":/select_three.png")<<QIcon(":/select_four.png")<<QIcon(":/select_five.png")<<QIcon(":/select_return.png");
    action_icons_<<QIcon(":/select_up.png")<<QIcon(":/select_down.png")<<QIcon(":/select_change.png")<<QIcon(":/select_return.png");

    re_attack_icons_<<QIcon(":/recommend_balance.png")<<QIcon(":/recommend_radical.png")<<QIcon(":/recommend_conservative.png");
    re_defend_icons_<<QIcon(":/recommend_regional.png")<<QIcon(":/recommend_man2man.png")<<QIcon(":/recommend_focus.png");

    //initialization information show
    action_shows_<<ui->show_action_1<<ui->show_action_2<<ui->show_action_3<<ui->show_action_4<<ui->show_action_5;
    role_shows_<<ui->show_role_1<<ui->show_role_2<<ui->show_role_3<<ui->show_role_4<<ui->show_role_5;
    battery_shows_<<ui->show_battery_1<<ui->show_battery_2<<ui->show_battery_3<<ui->show_battery_4<<ui->show_battery_5;
    robotinfo_shows_<<ui->nubot_info_1<<ui->nubot_info_2<<ui->nubot_info_3<<ui->nubot_info_4<<ui->nubot_info_5;

    for(int i=0;i<TEAM_NUM;i++)
        robotinfo_shows_[i]->setEnabled(false);

    //initialization the flags
    ui->selection_show->setText("NO_STRATEGY");
    ui->recommend_show->setText("NO_STRATEGY");
    BCI2robot_info_->selectStrategy=NO_STRATEGY;
    BCI2robot_info_->isNeworder=false;
    BCI2robot_info_->isReset=false;
    robot2BCI_info_->isNewstrategy_=false;
    BCI_signal_info_->select_signal=false;
    BCI_signal_info_->result_signal=-1;
    current_layer_=STRATEGY_L;
    changeLayer_(current_layer_);
    isConnect_BCI_=true;
    isStart_BCI_=false;
    holdNew_order_=false;

    int _frequency[6]={12,8,10,7,11,9};
    for(int i=0;i<6;i++)
        frequency_[i]=_frequency[i];
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    //draw robots
    for(int i=0;i<TEAM_NUM;i++)
    {
        nubot::Robot _tmp_robotifo=robot2BCI_info_->RobotInfo_[i];
        robot_[i]->setPos(_tmp_robotifo.robot_loc_.x_*WIDTH+OFFSET_R_W,-_tmp_robotifo.robot_loc_.y_*HEIGHT+OFFSET_R_H);
        robot_[i]->setTransformOriginPoint(25,25);
        robot_[i]->setRotation(-_tmp_robotifo.robot_head_.degree());
    }

    //draw ball
    ball_->setPos(robot2BCI_info_->BallInfo_.ball_global_loc_.x_*WIDTH+OFFSET_B_W,-robot2BCI_info_->BallInfo_.ball_global_loc_.y_*HEIGHT+OFFSET_B_H);

    //draw opponents and their velocity
    for(unsigned i=0;i<robot2BCI_info_->OpponentInfo_.size();i++)
        if(robot2BCI_info_->OpponentInfo_[i].is_robot_valid_)
        {
            nubot::Robot _tmp_opponentifo=robot2BCI_info_->OpponentInfo_[i];
            opponent_[i]->setPos(_tmp_opponentifo.robot_loc_.x_*WIDTH+OFFSET_R_W,-_tmp_opponentifo.robot_loc_.y_*HEIGHT+OFFSET_R_H);
            opponent_[i]->setTransformOriginPoint(25,25);
            opponent_[i]->setRotation(-_tmp_opponentifo.robot_head_.degree());

//            velocity_[i]->setPos(_tmp_opponentifo.robot_loc_.x_*WIDTH+OFFSET_V_W,-_tmp_opponentifo.robot_loc_.y_*HEIGHT+OFFSET_V_H);
//            velocity_[i]->setTransformOriginPoint(-29,20);
//            velocity_[i]->setRotation(-_tmp_opponentifo.robot_head_.degree());
        }
}

//"R" can reset the positions of opponents and ball for next round
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    bool _tmp_all_location=true;
    for(int i=0; i<TEAM_NUM; i++)
        if(robot2BCI_info_->RobotInfo_[i].current_role_!=SUBSTITUTE)
            _tmp_all_location=_tmp_all_location&&robot2BCI_info_->RobotInfo_[i].is_robot_location_;

    if(event->key()==Qt::Key_R&&_tmp_all_location)
         BCI2robot_info_->isReset=true;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    ros::shutdown();
}

void MainWindow::timerUpdate()
{
    static int _update_count=0;

    if(_update_count==6)
    {
        _update_count=0;
        static QString _strategy_string[20] = {"NO_STRATEGY","STRATEGY_ATTACK","STRATEGY_DEFEND","STRATEGY_ROBOT",
                                               "CONSERVATIVE","BALANCE","RADICAL",
                                               "ZONE","MARKING","FOCUS",
                                               "NUBOT_ONE","NUBOT_TWO","NUBOT_THREE","NUBOT_FOUR","NUBOT_FIVE",
                                               "UP_MAXSPEED","DOWN_MAXSPEED","CHANGE_ROBOT","RETURN"};
        static int _round_count=0;

        update();
        showRobotinfo_();
        if(holdNew_order_)
        {
            for(int i=0;i<select_buttons_.size();i++)
            {
                if(i==final_selection_&&!select_buttons_[i]->isEnabled())
                    select_buttons_[i]->setEnabled(true);
                else if(i!=final_selection_&&select_buttons_[i]->isEnabled())
                    select_buttons_[i]->setEnabled(false);
            }
            holdNeworder_();
            return;
        }
        if(isStart_BCI_)
        {
            if(BCI_signal_info_->select_signal)
            {
                BCI_signal_info_->select_signal=false;
                changeLayer_(STRATEGY_L);
                select_signal_=true;
            }

            if(robot2BCI_info_->isNewstrategy_)
            {
                _round_count++;
                select_signal_=true;
                p300_count_=0;
                ssvep_count_=0;
                ui->recommend_show->setText(_strategy_string[int(robot2BCI_info_->recommend_strategy_)]);
                ui->show_result->append(QString::number(_round_count)+"           "+_strategy_string[int(robot2BCI_info_->recommend_strategy_)].left(3)+"           ");

                if(robot2BCI_info_->recommend_strategy_>=CONSERVATIVE && robot2BCI_info_->recommend_strategy_<=RADICAL)
                {
                    current_layer_=ATTACK_L;
                    changeLayer_(current_layer_);
                    BCI2robot_info_->selectStrategy=STRATEGY_ATTACK;
                }
                else if(robot2BCI_info_->recommend_strategy_>=REGIONAL && robot2BCI_info_->recommend_strategy_<=FOCUS)
                {
                    current_layer_=DEFEND_L;
                    changeLayer_(current_layer_);
                    BCI2robot_info_->selectStrategy=STRATEGY_DEFEND;
                }

                robot2BCI_info_->isNewstrategy_=false;
            }
        }
        return;
    }

    _update_count++;
    if(isStart_BCI_)
    {
        if(ui->P300->isChecked()&&select_signal_)
        {
            if(p300_count_%P300_TIMER==0)
                p300Control_(p300_count_/P300_TIMER);
            p300_count_++;
            if(p300_count_==button_number_*P300_TIMER)
                p300_count_=0;

            if(BCI_signal_info_->result_signal>=0)
            {
                BCI_Info_(BCI_signal_info_->result_signal);
                BCI_signal_info_->result_signal=-1;
            }
        }
        else if(ui->SSVEP->isChecked()&&select_signal_)
        {
            ssvepControl_(ssvep_count_);
            ssvep_count_++;
            if(ssvep_count_==ssvep_max_count_)
                ssvep_count_=0;

            if(BCI_signal_info_->result_signal>=0)
            {
                BCI_Info_(BCI_signal_info_->result_signal);
                BCI_signal_info_->result_signal=-1;
            }
        }
    }
}

//connect BCI
void MainWindow::on_connect_clicked()
{
    if(!isConnect_BCI_)
    {
        QString IP=ui->IP_in->text();
//        qDebug()<<IP;
        quint16 prot=28097;
        tcpSocket_->abort();
        tcpSocket_->connectToHost(IP,prot);

        connect(tcpSocket_, SIGNAL(readyRead()), this, SLOT(OnReceive_()));
        ui->connect->setText("Disconnect");
        isConnect_BCI_=true;
        BCI2robot_info_->selectStrategy=STOPROBOT;

        ui->statusBar->showMessage("Try to connected BCI",1000);
    }
    else
    {
        tcpSocket_->disconnectFromHost();
        disconnect(tcpSocket_, SIGNAL(readyRead()), this, SLOT(OnReceive_()));
        ui->connect->setText("Connect BCI");
        isConnect_BCI_=false;
        holdNew_order_=false;
        ui->statusBar->showMessage("Disconnected",1000);

        BCI2robot_info_->selectStrategy=NO_STRATEGY;
        ui->selection_show->setText("NO_STRATEGY");
        current_layer_=STRATEGY_L;
        changeLayer_(current_layer_);
    }
}

//Obtained the BCI control order
void MainWindow::BCI_Info_(char bci_signal)
{
    qDebug()<<bci_signal;
    switch (bci_signal)
    {
    case SELECTION_ONE:
        switch (current_layer_)
        {
        case STRATEGY_L:
            current_layer_=ATTACK_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectStrategy=STRATEGY_ATTACK;
            ui->selection_show->setText("STRATEGY_ATTACK");
            break;

        case ATTACK_L:
            BCI2robot_info_->attackMode=BALANCE;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("BALANCE");
            ui->show_result->insertPlainText("BAL");
            final_selection_=0;
            holdNew_order_=true;
            break;

        case DEFEND_L:
            BCI2robot_info_->defendMode=REGIONAL;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("ZONE");
            ui->show_result->insertPlainText("ZON");
            final_selection_=0;
            holdNew_order_=true;
            break;

        case NUBOT_L:
            current_layer_=ACTION_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectRobot=NUBOT_ONE;
            ui->selection_show->setText("NUBOT_ONE");
            break;

        case ACTION_L:
            BCI2robot_info_->robotMode=UP_MAXSPEED;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("UP_MAXSPEED");
            final_selection_=0;
            holdNew_order_=true;
            break;
        }
        break;
    case SELECTION_TWO:
        switch (current_layer_)
        {
        case STRATEGY_L:
            current_layer_=DEFEND_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectStrategy=STRATEGY_DEFEND;
            ui->selection_show->setText("STRATEGY_DEFEND");
            break;

        case ATTACK_L:
            BCI2robot_info_->attackMode=RADICAL;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("RADICAL");
            ui->show_result->insertPlainText("RAD");
            final_selection_=1;
            holdNew_order_=true;
            break;

        case DEFEND_L:
            BCI2robot_info_->defendMode=MAN2MAN;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("MARKING");
            ui->show_result->insertPlainText("MAR");
            final_selection_=1;
            holdNew_order_=true;
            break;

        case NUBOT_L:
            current_layer_=ACTION_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectRobot=NUBOT_TWO;
            ui->selection_show->setText("NUBOT_TWO");
            break;

        case ACTION_L:
            BCI2robot_info_->robotMode=DOWN_MAXSPEED;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("DOWN_MAXSPEED");
            final_selection_=1;
            holdNew_order_=true;
            break;
        }
        break;
    case SELECTION_THREE:
        switch (current_layer_)
        {
        case STRATEGY_L:
            current_layer_=NUBOT_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectStrategy=STRATEGY_ROBOT;
            ui->selection_show->setText("STRATEGY_ROBOT");
            break;

        case ATTACK_L:
            BCI2robot_info_->attackMode=CONSERVATIVE;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("CONSERVATIVE");
            ui->show_result->insertPlainText("CON");
            final_selection_=2;
            holdNew_order_=true;
            break;

        case DEFEND_L:
            BCI2robot_info_->defendMode=FOCUS;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("FOCUS");
            ui->show_result->insertPlainText("FOC");
            final_selection_=2;
            holdNew_order_=true;
            break;

        case NUBOT_L:
            current_layer_=ACTION_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectRobot=NUBOT_THREE;
            ui->selection_show->setText("NUBOT_THREE");
            break;

        case ACTION_L:
            BCI2robot_info_->robotMode=CHANGE_ROBOT;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("CHANGE_ROBOT");
            final_selection_=2;
            holdNew_order_=true;
            break;
        }
        break;
    case SELECTION_FOUR:
        switch (current_layer_)
        {
        case STRATEGY_L:
            BCI2robot_info_->selectStrategy=STOPROBOT;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("STOPROBOT");
            final_selection_=3;
            holdNew_order_=true;
            break;

        case ATTACK_L:
            current_layer_=STRATEGY_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->attackMode=NO_STRATEGY;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("NO_STRATEGY");
            break;

        case DEFEND_L:
            current_layer_=STRATEGY_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->defendMode=NO_STRATEGY;
            BCI2robot_info_->isNeworder=true;
            ui->selection_show->setText("NO_STRATEGY");
            break;

        case NUBOT_L:
            current_layer_=ACTION_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectRobot=NUBOT_FOUR;
            ui->selection_show->setText("NUBOT_FOUR");
            break;

        case ACTION_L:
            current_layer_=NUBOT_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->robotMode=NO_STRATEGY;
            ui->selection_show->setText("NO_STRATEGY");
            break;
        }
        break;
    case SELECTION_FIVE:
        switch (current_layer_)
        {
        case NUBOT_L:
            current_layer_=ACTION_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectRobot=NUBOT_FIVE;
            ui->selection_show->setText("NUBOT_FIVE");
            break;
        }
        break;
    case SELECTION_SIX:
        switch (current_layer_)
        {
        case NUBOT_L:
            current_layer_=STRATEGY_L;
            changeLayer_(current_layer_);
            BCI2robot_info_->selectRobot=NO_STRATEGY;
            ui->selection_show->setText("NO_STRATEGY");
            break;
        }
        break;
    }
    if(BCI2robot_info_->isNeworder)
        select_signal_=false;
}

void MainWindow::OnReceive_()
{
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    QByteArray *buffer = buffers.value(socket);

    while (socket->bytesAvailable() > 0)
    {
        buffer->clear();
        buffer->append(socket->readAll());
        for(int i=0;i<buffer->size();i++)
        {
            BCI_Info_(buffer->data()[i]);
            qDebug()<<buffer->size();
        }
    }
}

void MainWindow::displayError_(QAbstractSocket::SocketError)
{
    QString error = tcpSocket_->errorString();
    QMessageBox::information(this,"Notice",error,QMessageBox::Ok,QMessageBox::Ok);
    tcpSocket_->close();

    ui->connect->setText("Connect");
    isConnect_BCI_=false;

    ui->statusBar->showMessage("Disconnected",1000);
}

void MainWindow::ssvepControl_(int button_n)
{
    for(int i=0;i<button_number_;i++)
    {
        if(button_n%frequency_[i]==0)
        {
            if(select_buttons_[i]->isEnabled())
                select_buttons_[i]->setEnabled(false);
            else
                select_buttons_[i]->setEnabled(true);
        }
    }
}

void MainWindow::p300Control_(int button_n)
{
    for(int i=0;i<button_number_;i++)
    {
        if(i!=button_n&&select_buttons_[i]->isEnabled())
            select_buttons_[i]->setEnabled(false);
        else if(i==button_n&&!select_buttons_[i]->isEnabled())
            select_buttons_[i]->setEnabled(true);
    }
}

void MainWindow::changeLayer_(char layer)
{
    switch (layer)
    {
    case STRATEGY_L:
        for(int i=0;i<select_buttons_.size();i++)
        {
            if(i<strategy_icons_.size())
            {
                select_buttons_[i]->setIcon(strategy_icons_[i]);
                select_buttons_[i]->setEnabled(true);
            }
            else
            {
                select_buttons_[i]->setIcon(QIcon());
                select_buttons_[i]->setEnabled(false);
            }
        }
        button_number_=short(strategy_icons_.size());
        break;

    case ATTACK_L:
        for(int i=0;i<select_buttons_.size();i++)
        {
            if(i<attack_icons_.size())
            {
                if(robot2BCI_info_->isNewstrategy_&&i==0&&robot2BCI_info_->recommend_strategy_==BALANCE)
                {
                    select_buttons_[i]->setIcon(re_attack_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
                else if(robot2BCI_info_->isNewstrategy_&&i==1&&robot2BCI_info_->recommend_strategy_==RADICAL)
                {
                    select_buttons_[i]->setIcon(re_attack_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
                else if(robot2BCI_info_->isNewstrategy_&&i==2&&robot2BCI_info_->recommend_strategy_==CONSERVATIVE)
                {
                    select_buttons_[i]->setIcon(re_attack_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
                else
                {
                    select_buttons_[i]->setIcon(attack_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
            }
            else
            {
                select_buttons_[i]->setIcon(QIcon());
                select_buttons_[i]->setEnabled(false);
            }
        }
        button_number_=short(attack_icons_.size());
        break;

    case DEFEND_L:
        for(int i=0;i<select_buttons_.size();i++)
        {
            if(i<defend_icons_.size())
            {
                if(robot2BCI_info_->isNewstrategy_&&i==0&&robot2BCI_info_->recommend_strategy_==REGIONAL)
                {
                    select_buttons_[i]->setIcon(re_defend_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
                else if(robot2BCI_info_->isNewstrategy_&&i==1&&robot2BCI_info_->recommend_strategy_==MAN2MAN)
                {
                    select_buttons_[i]->setIcon(re_defend_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
                else if(robot2BCI_info_->isNewstrategy_&&i==2&&robot2BCI_info_->recommend_strategy_==FOCUS)
                {
                    select_buttons_[i]->setIcon(re_defend_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
                else
                {
                    select_buttons_[i]->setIcon(defend_icons_[i]);
                    select_buttons_[i]->setEnabled(true);
                }
            }
            else
            {
                select_buttons_[i]->setIcon(QIcon());
                select_buttons_[i]->setEnabled(false);
            }
        }
        button_number_=short(defend_icons_.size());
        break;

    case NUBOT_L:
        for(int i=0;i<select_buttons_.size();i++)
        {
            if(i<nubot_icons_.size())
            {
                select_buttons_[i]->setIcon(nubot_icons_[i]);
                select_buttons_[i]->setEnabled(true);
            }
            else
            {
                select_buttons_[i]->setIcon(QIcon());
                select_buttons_[i]->setEnabled(false);
            }
        }
        button_number_=short(nubot_icons_.size());
        break;

    case ACTION_L:
        for(int i=0;i<select_buttons_.size();i++)
        {
            if(i<action_icons_.size())
            {
                select_buttons_[i]->setIcon(action_icons_[i]);
                select_buttons_[i]->setEnabled(true);
            }
            else
            {
                select_buttons_[i]->setIcon(QIcon());
                select_buttons_[i]->setEnabled(false);
            }
        }
        button_number_=short(action_icons_.size());
        break;
    }

    p300_count_=0;
    ssvep_count_=0;
    ssvep_max_count_=1;
    for(int i=0;i<button_number_;i++)
        ssvep_max_count_=ssvep_max_count_*frequency_[i];
}

void MainWindow::holdNeworder_()
{
    static int _count=0;
    _count++;
    if(_count==RESET_TIMER)
    {
        BCI2robot_info_->selectStrategy=NO_STRATEGY;
        ui->selection_show->setText("NO_STRATEGY");
        current_layer_=STRATEGY_L;
        changeLayer_(current_layer_);
        _count=0;
        holdNew_order_=false;
    }
}

void MainWindow::showRobotinfo_()
{
    static QString _role_string[6] = {"NOROLE","ACTIVE","PASSIVE","ASSISTANT","MIDFIELD","SUBSTITUTE"};
    static QString _action_string[9] = {"No_Action","Stucked","CanNotSeeBall","CatchBall","AvoidObs","Pass","Receive","Shoot","Positioned"};
    static bool _ourball=false;
    for(int i=0;i<TEAM_NUM;i++)
    {
        if(robot2BCI_info_->RobotInfo_[i].is_robot_valid_)
        {
            if(!robotinfo_shows_[i]->isEnabled())
                robotinfo_shows_[i]->setEnabled(true);
            action_shows_[i]->setText(_action_string[int(robot2BCI_info_->RobotInfo_[i].current_action_)]);
            role_shows_[i]->setText(_role_string[int(robot2BCI_info_->RobotInfo_[i].current_role_)]);
            battery_shows_[i]->setValue(int(robot2BCI_info_->RobotInfo_[i].battery_state_));
            if(robot2BCI_info_->RobotInfo_[i].is_robot_dribble_&&!_ourball)
                ui->statusBar->showMessage("NuBot get the ball!",2000);
        }
        else if(robotinfo_shows_[i]->isEnabled())
        {
            action_shows_[i]->setText("SUBSTITUTE");
            role_shows_[i]->setText(_role_string[0]);
            battery_shows_[i]->setValue(100);
            robotinfo_shows_[i]->setEnabled(false);
        }
    }
}

void MainWindow::on_start_clicked()
{
    if(!isStart_BCI_)
    {
        if(!ui->P300->isChecked()&&!ui->SSVEP->isChecked())
        {
            isStart_BCI_=true;
            ui->start->setText("STOP");
            ui->P300->setEnabled(false);
            ui->SSVEP->setEnabled(false);
        }
        else if(isConnect_BCI_)
        {
            isStart_BCI_=true;
            ui->start->setText("STOP");
            ui->P300->setEnabled(false);
            ui->SSVEP->setEnabled(false);
            for(int i=0;i<select_buttons_.size();i++)
                select_buttons_[i]->setEnabled(false);
        }
        else
            QMessageBox::information(this,"Notice","Please connect BCI first!",QMessageBox::Ok,QMessageBox::Ok);
    }
    else
    {
        isStart_BCI_=false;
        holdNew_order_=false;
        ui->start->setText("START");
        ui->P300->setEnabled(true);
        ui->SSVEP->setEnabled(true);

        for(int i=0;i<select_buttons_.size();i++)
            select_buttons_[i]->setEnabled(false);
        BCI2robot_info_->selectStrategy=NO_STRATEGY;
        ui->selection_show->setText("NO_STRATEGY");
        current_layer_=STRATEGY_L;
        changeLayer_(current_layer_);
    }
    BCI2robot_info_->isNeworder=true;
    BCI2robot_info_->stopORstart=isStart_BCI_;
}

void MainWindow::on_selection1_clicked()
{
    switch (current_layer_)
    {
    case STRATEGY_L:
        current_layer_=ATTACK_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectStrategy=STRATEGY_ATTACK;
        ui->selection_show->setText("STRATEGY_ATTACK");
        break;

    case ATTACK_L:
        BCI2robot_info_->attackMode=BALANCE;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("BALANCE");
        ui->show_result->insertPlainText("BAL");
        final_selection_=0;
        holdNew_order_=true;
        break;

    case DEFEND_L:
        BCI2robot_info_->defendMode=REGIONAL;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("ZONE");
        ui->show_result->insertPlainText("ZON");
        final_selection_=0;
        holdNew_order_=true;
        break;

    case NUBOT_L:
        current_layer_=ACTION_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectRobot=NUBOT_ONE;
        ui->selection_show->setText("NUBOT_ONE");
        break;

    case ACTION_L:
        BCI2robot_info_->robotMode=UP_MAXSPEED;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("UP_MAXSPEED");
        final_selection_=0;
        holdNew_order_=true;
        break;
    }
}

void MainWindow::on_selection2_clicked()
{
    switch (current_layer_)
    {
    case STRATEGY_L:
        current_layer_=DEFEND_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectStrategy=STRATEGY_DEFEND;
        ui->selection_show->setText("UP_MAXSPEED");
        break;

    case ATTACK_L:
        BCI2robot_info_->attackMode=RADICAL;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("RADICAL");
        ui->show_result->insertPlainText("RAD");
        final_selection_=1;
        holdNew_order_=true;
        break;

    case DEFEND_L:
        BCI2robot_info_->defendMode=MAN2MAN;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("MARKING");
        ui->show_result->insertPlainText("MAR");
        final_selection_=1;
        holdNew_order_=true;
        break;

    case NUBOT_L:
        current_layer_=ACTION_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectRobot=NUBOT_TWO;
        ui->selection_show->setText("NUBOT_TWO");
        break;

    case ACTION_L:
        BCI2robot_info_->robotMode=DOWN_MAXSPEED;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("DOWN_MAXSPEED");
        final_selection_=1;
        holdNew_order_=true;
        break;
    }
}

void MainWindow::on_selection3_clicked()
{
    switch (current_layer_)
    {
    case STRATEGY_L:
        current_layer_=NUBOT_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectStrategy=STRATEGY_ROBOT;
        ui->selection_show->setText("STRATEGY_ROBOT");
        break;

    case ATTACK_L:
        BCI2robot_info_->attackMode=CONSERVATIVE;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("CONSERVATIVE");
        ui->show_result->insertPlainText("CON");
        final_selection_=2;
        holdNew_order_=true;
        break;

    case DEFEND_L:
        BCI2robot_info_->defendMode=FOCUS;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("FOCUS");
        ui->show_result->insertPlainText("FOC");
        final_selection_=2;
        holdNew_order_=true;
        break;

    case NUBOT_L:
        current_layer_=ACTION_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectRobot=NUBOT_THREE;
        ui->selection_show->setText("NUBOT_THREE");
        break;

    case ACTION_L:
        BCI2robot_info_->robotMode=CHANGE_ROBOT;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("CHANGE_ROBOT");
        final_selection_=2;
        holdNew_order_=true;
        break;
    }
}

void MainWindow::on_selection4_clicked()
{
    switch (current_layer_)
    {
    case STRATEGY_L:
        BCI2robot_info_->selectStrategy=STOPROBOT;
        BCI2robot_info_->isNeworder=true;
        ui->selection_show->setText("STOPROBOT");
        final_selection_=3;
        holdNew_order_=true;
        break;

    case ATTACK_L:
        current_layer_=STRATEGY_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->attackMode=NO_STRATEGY;
        ui->selection_show->setText("NO_STRATEGY");
        break;

    case DEFEND_L:
        current_layer_=STRATEGY_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->defendMode=NO_STRATEGY;
        ui->selection_show->setText("NO_STRATEGY");
        break;

    case NUBOT_L:
        current_layer_=ACTION_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectRobot=NUBOT_FOUR;
        ui->selection_show->setText("NUBOT_FOUR");
        break;

    case ACTION_L:
        current_layer_=NUBOT_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->robotMode=NO_STRATEGY;
        ui->selection_show->setText("NO_STRATEGY");
        break;
    }
}

void MainWindow::on_selection5_clicked()
{
    switch (current_layer_)
    {
    case NUBOT_L:
        current_layer_=ACTION_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectRobot=NUBOT_FIVE;
        ui->selection_show->setText("NUBOT_FIVE");
        break;
    }
}

void MainWindow::on_selection6_clicked()
{
    switch (current_layer_)
    {
    case NUBOT_L:
        current_layer_=STRATEGY_L;
        changeLayer_(current_layer_);
        BCI2robot_info_->selectRobot=NO_STRATEGY;
        ui->selection_show->setText("NO_STRATEGY");
        break;
    }
}
