#pragma once

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QtWidgets>
#include <QDoubleValidator>

//thanks to https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "localization_monitor/dataviewer.h"

#include <sstream>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
private:

    ros::NodeHandle nh_;

    Ui::MainWindow *ui;

    DataViewer *data_viewer_;

    QGridLayout *main_layout_;
    QWidget *main_widget_;

    QGroupBox *mcl_param_gb_;
    QHBoxLayout *mcl_param_layout_;
    QTabWidget *mcl_param_tab_;

    //Common param
    QHBoxLayout *common_param_layout_;
    QWidget *common_param_widget_;
    QLabel *num_particles_label_;
    QSpinBox *num_particles_sb_;
    QLabel *range_var_label_;
    QDoubleSpinBox *range_var_dsb_;
    QLabel *beam_var_label_;
    QDoubleSpinBox *beam_var_dsb_;

    //Motion param
    QHBoxLayout *motion_param_layout_;
    QWidget *motion_param_widget_;
    QLabel *alpha1_label_;
    QLineEdit *alpha1_lined_;
    QDoubleSpinBox *alpha1_dsb_;
    QLabel *alpha2_label_;
    QLineEdit *alpha2_lined_;
    QDoubleSpinBox *alpha2_dsb_;
    QLabel *alpha3_label_;
    QLineEdit *alpha3_lined_;
    QDoubleSpinBox *alpha3_dsb_;
    QLabel *alpha4_label_;
    QLineEdit *alpha4_lined_;
    QDoubleSpinBox *alpha4_dsb_;

    QGroupBox *misc_gb_;
    QGridLayout *misc_layout_;
    QPushButton *reset_particles_pb_;
    ros::Publisher reset_particles_pub_;
    QCheckBox *show_grid_pb_;
    QCheckBox *show_particles_pb_;

    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;        

    void settingWidgets();
    void settingActions();

    ros::Publisher save_params_pub_;
    void loadParams(){
        YAML::Node params_file;
        try{
            params_file = YAML::LoadFile(ros::package::getPath("alfarobi_vision") + "/config/v9_localization/saved_params.yaml");
        }catch(const std::exception &e){
            ROS_ERROR("localization_monitor: Unable to load params");
        }

        num_particles_sb_->setValue(params_file["num_particles"].as<int>());
        range_var_dsb_->setValue(params_file["range_var"].as<double>());
        beam_var_dsb_->setValue(params_file["beam_var"].as<double>());
        alpha1_lined_->setText(tr("%1").arg(params_file["alpha1"].as<double>()));
        alpha2_lined_->setText(tr("%1").arg(params_file["alpha2"].as<double>()));
        alpha3_lined_->setText(tr("%1").arg(params_file["alpha3"].as<double>()));
        alpha4_lined_->setText(tr("%1").arg(params_file["alpha4"].as<double>()));

    }
private slots:
    void updateDataViewer();
    void updateNumParticles(int _value);
    void updateAlphaGain();
    void updateVariance(double _value);
    void showGrid(int flag);
    void showParticles(int flag);
    void resetParticles();
};
