#include "localization_monitor/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    nh_(ros::this_node::getName()),
    ui(new Ui::MainWindow){

    ui->setupUi(this);

    std::stringstream icon_path;
    icon_path << ros::package::getPath("vision_monitor") << "/icon/logo_alfarobi.png";
    QIcon icon;
    icon.addFile(icon_path.str().c_str());
    this->setWindowIcon(icon);

    std::stringstream style_path;
    style_path << ros::package::getPath("vision_monitor") << "/style/vision_style.qss";
    QFile style_file(style_path.str().c_str());
    style_file.open(QFile::ReadOnly);
    QString read_style(style_file.readAll());
    this->setStyleSheet(read_style);

    main_layout_ = new QGridLayout;
    main_widget_ = new QWidget;

    settingWidgets();
    loadParams();
    settingActions();

    main_widget_->setLayout(main_layout_);

    this->setCentralWidget(main_widget_);
    this->setWindowTitle(tr("Localization Monitor"));
    this->removeToolBar(ui->mainToolBar);
    this->setMaximumSize(this->centralWidget()->width(),this->centralWidget()->height());    

    data_viewer_->processBegin();

    reset_particles_pub_ = nh_.advertise<std_msgs::Bool>("reset_particles",10);
    save_params_pub_ = nh_.advertise<std_msgs::Empty>("save_params",10);

}

MainWindow::~MainWindow(){
    delete ui;
}

void MainWindow::settingWidgets(){
    data_viewer_ = new DataViewer;
    data_viewer_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    data_viewer_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    data_viewer_->verticalScrollBar()->blockSignals(true);
    data_viewer_->horizontalScrollBar()->blockSignals(true);

    num_particles_label_ = new QLabel;
    num_particles_label_->setText(tr("Num. Particles : "));
    num_particles_sb_ = new QSpinBox;
    num_particles_sb_->setMaximum(999999);
    num_particles_sb_->setMinimum(50);
    num_particles_sb_->setValue(100);

    range_var_label_ = new QLabel;
    range_var_label_->setText(tr("Range Var. : "));
    range_var_dsb_ = new QDoubleSpinBox;
    range_var_dsb_->setMaximum(999999);
    range_var_dsb_->setMinimum(0.1);
    range_var_dsb_->setValue(40000);

    beam_var_label_ = new QLabel;
    beam_var_label_->setText(tr("Beam Var. : "));
    beam_var_dsb_ = new QDoubleSpinBox;
    beam_var_dsb_->setMaximum(999999);
    beam_var_dsb_->setMinimum(0.1);
    beam_var_dsb_->setValue(7225);

    common_param_layout_ = new QHBoxLayout;
    common_param_layout_->addWidget(num_particles_label_);
    common_param_layout_->addWidget(num_particles_sb_);
    common_param_layout_->addWidget(range_var_label_);
    common_param_layout_->addWidget(range_var_dsb_);
    common_param_layout_->addWidget(beam_var_label_);
    common_param_layout_->addWidget(beam_var_dsb_);
    common_param_layout_->addStretch(1);

    common_param_widget_ = new QWidget;
    common_param_widget_->setLayout(common_param_layout_);

    QDoubleValidator *dv = new QDoubleValidator(0.0,1.0,6);
    alpha1_label_ = new QLabel;
    alpha1_label_->setText(tr("Alpha 1 : "));
    alpha1_lined_ = new QLineEdit;
    alpha1_lined_->setValidator(dv);
//    alpha1_dsb_ = new QDoubleSpinBox;
//    alpha1_dsb_->setValue(1.0);

    alpha2_label_ = new QLabel;
    alpha2_label_->setText(tr("Alpha 2 : "));
    alpha2_lined_ = new QLineEdit;
    alpha2_lined_->setValidator(dv);
//    alpha2_dsb_ = new QDoubleSpinBox;
//    alpha2_dsb_->setValue(1.0);

    alpha3_label_ = new QLabel;
    alpha3_label_->setText(tr("Alpha 3 : "));
    alpha3_lined_ = new QLineEdit;
    alpha3_lined_->setValidator(dv);
//    alpha3_dsb_ = new QDoubleSpinBox;
//    alpha3_dsb_->setValue(1.0);

    alpha4_label_ = new QLabel;
    alpha4_label_->setText(tr("Alpha 4 : "));
    alpha4_lined_ = new QLineEdit;
    alpha4_lined_->setValidator(dv);
//    alpha4_dsb_ = new QDoubleSpinBox;
//    alpha4_dsb_->setValue(1.0);

    motion_param_layout_ = new QHBoxLayout;
    motion_param_layout_->addWidget(alpha1_label_);
    motion_param_layout_->addWidget(alpha1_lined_);
    motion_param_layout_->addWidget(alpha2_label_);
    motion_param_layout_->addWidget(alpha2_lined_);
    motion_param_layout_->addWidget(alpha3_label_);
    motion_param_layout_->addWidget(alpha3_lined_);
    motion_param_layout_->addWidget(alpha4_label_);
    motion_param_layout_->addWidget(alpha4_lined_);
    motion_param_layout_->addStretch(1);

    motion_param_widget_ = new QWidget;
    motion_param_widget_->setLayout(motion_param_layout_);

    mcl_param_tab_ = new QTabWidget;
    mcl_param_tab_->addTab(common_param_widget_,tr("Common"));
    mcl_param_tab_->addTab(motion_param_widget_,tr("Motion"));
    mcl_param_tab_->setWindowTitle(tr("MCL Parameters"));

    mcl_param_layout_ = new QHBoxLayout;
    mcl_param_layout_->addWidget(mcl_param_tab_);
    mcl_param_layout_->addStretch(1);
    mcl_param_gb_= new QGroupBox;
    mcl_param_gb_->setLayout(mcl_param_layout_);
    mcl_param_gb_->setTitle(tr("MCL Parameters"));

    //===============

    reset_particles_pb_ = new QPushButton;
    reset_particles_pb_->setText(tr("Reset Particles"));

    show_grid_pb_ = new QCheckBox;
    show_grid_pb_->setText(tr("Show Grid"));

    show_particles_pb_ = new QCheckBox;
    show_particles_pb_->setText(tr("Show Particles"));

    misc_layout_ = new QGridLayout;
    misc_layout_->addWidget(reset_particles_pb_,0,0,1,1);
    misc_layout_->addWidget(show_grid_pb_,1,0,1,1);
    misc_layout_->addWidget(show_particles_pb_,2,0,1,1);
//    misc_layout_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding, QSizePolicy::Expanding),3,0);

    misc_gb_ = new QGroupBox;
    misc_gb_->setTitle(tr("Misc."));
    misc_gb_->setLayout(misc_layout_);

    main_layout_->addWidget(data_viewer_,0,0,1,2);
    main_layout_->addWidget(mcl_param_gb_,1,0,1,1);
    main_layout_->addWidget(misc_gb_,1,1,1,1);
    main_layout_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),2,2,1,1);
}

void MainWindow::settingActions(){
    connect(data_viewer_,SIGNAL(updateViewer()),this,SLOT(updateDataViewer()));
    connect(num_particles_sb_,SIGNAL(valueChanged(int)),this,SLOT(updateNumParticles(int)));
    connect(alpha1_lined_,SIGNAL(returnPressed()),this,SLOT(updateAlphaGain()));
    connect(alpha2_lined_,SIGNAL(returnPressed()),this,SLOT(updateAlphaGain()));
    connect(alpha3_lined_,SIGNAL(returnPressed()),this,SLOT(updateAlphaGain()));
    connect(alpha4_lined_,SIGNAL(returnPressed()),this,SLOT(updateAlphaGain()));
    connect(range_var_dsb_,SIGNAL(valueChanged(double)),this,SLOT(updateVariance(double)));
    connect(beam_var_dsb_,SIGNAL(valueChanged(double)),this,SLOT(updateVariance(double)));
    connect(show_grid_pb_,SIGNAL(stateChanged(int)),this,SLOT(showGrid(int)));
    connect(show_particles_pb_,SIGNAL(stateChanged(int)),this,SLOT(showParticles(int)));
    connect(reset_particles_pb_,SIGNAL(clicked(bool)),this,SLOT(resetParticles()));
}

void MainWindow::updateDataViewer(){
    data_viewer_->updateData();
}

void MainWindow::updateNumParticles(int _value){
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;
    int_param.name = "num_particles";
    int_param.value = _value;
    conf.ints.push_back(int_param);
    srv_req_.config = conf;
    ros::service::call("/v9_localization_node/set_parameters", srv_req_,srv_resp_);
}

void MainWindow::updateAlphaGain(){
    QObject *object = sender();

    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    if(object == alpha1_lined_){
        double_param.name = "alpha1";
        double_param.value = alpha1_lined_->text().toDouble();
    }else if(object == alpha2_lined_){
        double_param.name = "alpha2";
        double_param.value = alpha2_lined_->text().toDouble();
    }else if(object == alpha3_lined_){
        double_param.name = "alpha3";
        double_param.value = alpha3_lined_->text().toDouble();
    }else if(object == alpha4_lined_){
        double_param.name = "alpha4";
        double_param.value = alpha4_lined_->text().toDouble();
    }

    conf.doubles.push_back(double_param);
    srv_req_.config = conf;
    ros::service::call("/v9_localization_node/set_parameters", srv_req_,srv_resp_);

}

void MainWindow::updateVariance(double _value){
    QObject *object = sender();

    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    if(object == range_var_dsb_){
        double_param.name = "range_var";
        double_param.value = _value;
    }else if(object == beam_var_dsb_){
        double_param.name = "beam_var";
        double_param.value = _value;
    }

    conf.doubles.push_back(double_param);
    srv_req_.config = conf;
    ros::service::call("/v9_localization_node/set_parameters", srv_req_,srv_resp_);
}

void MainWindow::showGrid(int flag){
    data_viewer_->show_grid_ = (bool)flag;
}

void MainWindow::showParticles(int flag){
    data_viewer_->show_particles_ = (bool)flag;
}

void MainWindow::resetParticles(){
    std_msgs::Bool reset_msg;
    reset_msg.data = true;
    reset_particles_pub_.publish(reset_msg);
}
