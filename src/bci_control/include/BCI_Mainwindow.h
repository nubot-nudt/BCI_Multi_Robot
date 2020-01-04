#ifndef BCI_WIDGET_H
#define BCI_WIDGET_H

#include <QMainWindow>
#include <QString>
#include <QDir>
#include <QCloseEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QHostAddress>
#include <QTime>
#include <QTimer>
#include <QVector>
#include <QLineEdit>
#include <QProgressBar>
#include <QGroupBox>
#include <QRadioButton>
#include <QWaitCondition>
#include <QPixmap>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <robot2BCI.h>
#include <QMessageBox>
#include <qgraphicsscene.h>
#include <qgraphicsview.h>
#include <QGraphicsPixmapItem>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>

#define  WIDTH    900/1800
#define  HEIGHT   600/1200
#define  OFFSET_B_W  481
#define  OFFSET_B_H  324
#define  OFFSET_R_W  476
#define  OFFSET_R_H  319
#define  OFFSET_V_W  530
#define  OFFSET_V_H  324

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(nubot::interface_Info & robot2BCI, nubot::BCIcontrol_Info & BCI2robot, nubot::BCI_signal & BCIsignal,
                        QWidget *parent = nullptr);
    ~MainWindow();
    void paintEvent(QPaintEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent *event);

    void  BCI_Info_(char bci_signal);
    void  buttonDelay_();
    void  ssvepControl_(int button_n);
    void  p300Control_(int button_n);
    void  showRobotinfo_();
    void  changeLayer_(char layer);
    void  holdNeworder_();

public:
    nubot::interface_Info  * robot2BCI_info_;
    nubot::BCIcontrol_Info * BCI2robot_info_;
    nubot::BCI_signal      * BCI_signal_info_;

    QPixmap field_img_;
    QPixmap robot_img_[TEAM_NUM];
    QPixmap ball_img_;
    QPixmap opponent_img_[OPP_TEAM_NUM];
    QPixmap velocity_img_;

    //scene and items
    QGraphicsScene *scene_;
    QGraphicsPixmapItem *field_;
    QGraphicsPixmapItem *ball_;
    QGraphicsPixmapItem *velocity_[OPP_TEAM_NUM];
    QGraphicsPixmapItem *robot_[TEAM_NUM];
    QGraphicsPixmapItem *opponent_[OPP_TEAM_NUM];

    //button icons for each layer
    QList<QPushButton *> select_buttons_;
    QList<QIcon> strategy_icons_;
    QList<QIcon> attack_icons_;
    QList<QIcon> defend_icons_;
    QList<QIcon> nubot_icons_;
    QList<QIcon> action_icons_;
    QList<QIcon> re_attack_icons_;
    QList<QIcon> re_defend_icons_;

    //robot information
    QList<QLineEdit *>    action_shows_;
    QList<QLineEdit *>    role_shows_;
    QList<QProgressBar *> battery_shows_;
    QList<QGroupBox *>    robotinfo_shows_;

    QTcpSocket *tcpSocket_;
    QHash<QTcpSocket*, QByteArray*> buffers;
    QHash<QTcpSocket*, qint32*> sizes;

private:
    Ui::MainWindow *ui;
    bool  isConnect_BCI_;
    bool  isStart_BCI_;
    bool  holdNew_order_;
    bool  select_signal_;
    short button_number_;
    int   p300_count_;
    int   ssvep_count_;
    int   ssvep_max_count_;
    int   frequency_[6];
    char  current_layer_;
    char  final_selection_;

private slots:
    void timerUpdate();
    void OnReceive_();
    void displayError_(QAbstractSocket::SocketError socketError);
    void on_connect_clicked();
    void on_start_clicked();
    void on_selection1_clicked();
    void on_selection2_clicked();
    void on_selection3_clicked();
    void on_selection4_clicked();
    void on_selection5_clicked();
    void on_selection6_clicked();
};


#endif // BCI_WIDGET_H
