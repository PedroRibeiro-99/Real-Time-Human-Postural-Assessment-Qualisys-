#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qualisys_data_streaming.h"
#include "segment.h"
#include "joint.h"
#include "plots_interface.h"
#include "ros_communication.h"
#include "rula.h"
#include <iostream>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
  void computeSegmentsRefFrames(Segment &segment, int frame);
  void computeJointsAngles();
  void computeRulaEvaluation(Rula &rRULA, Rula &lRULA, vector<int> &rulaStatusBuffer);
  void execFrameData(int frame, ros_communication &ros_handler);
  ~MainWindow();

private Q_SLOTS:
  void on_radioButton_loadFile_clicked();
  void on_radioButton_streamData_clicked();
  void setUIStreamingState(bool state);
  void setUIDataFileState(bool state);
  void setUIExecMoveState(bool state);
  void on_pushButton_loadfile_clicked();
  void on_pushButton_plots_clicked();
  bool verifySimulationEnvironment();
  void initSimulationEnvironment(ros::NodeHandle &n,ros::ServiceClient &rosClient);
  void on_pushButton_pauseMovement_clicked();
  void on_pushButton_resumeMovement_clicked();
  void on_pushButton_stopMovement_clicked();
  void on_pushButton_execMovement_clicked();
  void on_pushButton_execFrame_clicked();

  void on_pushButton_connectServer_clicked();

  void on_pushButton_disconnectServer_clicked();

private:
  int argc; char** argv;
  Ui::MainWindow *ui;
  QualysisMoveData qData;
  QualisysDataStreaming qStreaming;
  Segment sR_arm, sR_forearm, sR_wrist, sL_arm, sL_forearm, sL_wrist, s_neck, s_trunk;
  vector<Segment*> segments_buffer;
  Joint jR_arm, jR_forearm, jR_wrist,jL_arm, jL_forearm, jL_wrist, j_neck, j_trunk;
  vector<Joint*> joints_buffer;
  bool rightArmAssessmentAvailable;
  bool leftArmAssessmentAvailable;
  bool abortSimulation;
  bool pausedSimulation;
  bool serverConnected;
  PlotsInterface plot_interface;
};

#endif // MAINWINDOW_H
