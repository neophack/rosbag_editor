#include "rosbag_editor.h"
#include "ui_rosbag_editor.h"

#include <topic_tools/shape_shifter.h>
#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/filesystem.hpp>

#include <QDir>
#include <QFile>

#include <QString>
#include <QFileDialog>
#include <QDateTime>
#include <QSettings>
#include <QFont>
#include <QDateTimeEdit>
#include <QMessageBox>
#include <QItemSelectionModel>
#include <QPushButton>
#include <QListWidget>
#include <QStatusBar>
#include <QFileInfo>
#include <QLineEdit>
#include <QCheckBox>
#include <QProgressDialog>

#include "filter_frames.h"

#include <velodyne_pointcloud/rawdata.h>

RosbagEditor::RosbagEditor(QWidget *parent) : QMainWindow(parent),
                                              ui(new Ui::RosbagEditor)
{
  QApplication::setWindowIcon(QIcon("://rosbag_editor.png"));
  ui->setupUi(this);

  QSettings settings("DavideFaconti", "rosbag_editor");
  _previous_load_path = settings.value("RosbagEditor/prevLoadPath", QDir::currentPath()).toString();
  _previous_save_path = settings.value("RosbagEditor/prevSavePath", _previous_load_path).toString();

  ui->radioNoCompression->setChecked(true);

  restoreGeometry(settings.value("RosbagEditor/geometry").toByteArray());
  restoreState(settings.value("RosbagEditor/windowState").toByteArray());

  connect(ui->tableWidgetInput, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(PerviewSlot(int, int)));
  connect(ui->horizontalSlider, &QSlider::valueChanged, this, &RosbagEditor::processSlider);
  // connect(ui->dateTimeEditCurrent, &QDoubleSpinBox::valueChanged, this, &RosbagEditor::updateSlider);

  avcodec_register_all();
  av_log_set_level(AV_LOG_QUIET);

  AVCodec *decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!decoder)
    throw std::runtime_error("H264 decoding not supported in this build of ffmpeg");

  g_codec = avcodec_alloc_context3(decoder);

  g_codec->flags |= CODEC_FLAG_LOW_DELAY;
  g_codec->flags2 |= CODEC_FLAG2_SHOW_ALL;

  g_codec->thread_type = 0;

  if (avcodec_open2(g_codec, decoder, 0) != 0)
    throw std::runtime_error("Could not open decoder");
}

void RosbagEditor::closeEvent(QCloseEvent *event)
{
  QSettings settings("DavideFaconti", "rosbag_editor");
  settings.setValue("RosbagEditor/geometry", saveGeometry());
  settings.setValue("RosbagEditor/windowState", saveState());

  if (img_viewer_)
  {
    img_viewer_->close();
    img_viewer_.release();
  }
  if (pc_viewer_)
  {
    pc_viewer_->close();
    img_viewer_.release();
  }
  QMainWindow::closeEvent(event);
}

RosbagEditor::~RosbagEditor()
{
  delete ui;
}

void RosbagEditor::on_pushButtonLoad_pressed()
{
  QString filename = QFileDialog::getOpenFileName(this,
                                                  tr("Open Rosbag"), _previous_load_path, tr("Rosbag Files (*.bag)"));

  if (filename.isEmpty())
  {
    return;
  }

  try
  {
    ui->tableWidgetInput->setRowCount(0);
    ui->tableWidgetOutput->setRowCount(0);
    _bag.close();
    _bag.open(filename.toStdString());
  }
  catch (rosbag::BagException &ex)
  {
    QMessageBox::warning(this, "Error opening the rosbag",
                         tr("rosbag::open thrown an exception: %1\n").arg(ex.what()));
    return;
  }

  QSettings settings;
  settings.setValue("RosbagEditor/prevLoadPath", QFileInfo(filename).absolutePath());

  ui->statusbar->showMessage(tr("File loaded: %1").arg(filename));

  _loade_filename = filename;

  rosbag::View bag_view(_bag);
  auto connections = bag_view.getConnections();

  ui->tableWidgetInput->setRowCount(connections.size());
  ui->tableWidgetInput->setColumnCount(2);
  ui->tableWidgetInput->setEnabled(true);

  QDateTime datetime_begin = QDateTime::fromMSecsSinceEpoch(bag_view.getBeginTime().toSec() * 1000);
  ui->dateTimeInputBegin->setDateTime(datetime_begin);

  QDateTime datetime_end = QDateTime::fromMSecsSinceEpoch(bag_view.getEndTime().toSec() * 1000);
  ui->dateTimeInputEnd->setDateTime(datetime_end);

  // std::cout<<bag_view.size()<<std::endl;
  ui->labelCount->setText(QString::number(bag_view.size()));
  ui->horizontalSlider->setMaximum(bag_view.size() - 1);
  sec_begin_ = bag_view.getBeginTime().toSec();
  sec_end_ = bag_view.getEndTime().toSec();

  std::map<QString, QString> connections_ordered;

  for (std::size_t i = 0; i < connections.size(); i++)
  {
    const rosbag::ConnectionInfo *connection = connections[i];
    connections_ordered.insert(std::make_pair(QString::fromStdString(connection->topic),
                                              QString::fromStdString(connection->datatype)));
  }

  int row = 0;
  for (const auto conn : connections_ordered)
  {
    auto type_item = new QTableWidgetItem(conn.second);
    QFont font = type_item->font();
    font.setPointSize(8);
    font.setItalic(true);
    type_item->setFont(font);

    ui->tableWidgetInput->setItem(row, 0, new QTableWidgetItem(conn.first));
    ui->tableWidgetInput->setItem(row, 1, type_item);
    row++;
  }
  changeEnabledWidgets();
}

void RosbagEditor::changeEnabledWidgets()
{
  ui->pushButtonMove->setEnabled(ui->tableWidgetInput->selectionModel()->selectedRows().count() > 0);
  bool output = (ui->tableWidgetInput->rowCount() > 0);
  ui->tableWidgetOutput->setEnabled(output);
  ui->dateTimeOutputBegin->setEnabled(output);
  ui->dateTimeOutputEnd->setEnabled(output);
  ui->pushButtonSave->setEnabled(output);
  ui->pushButtonConvert->setEnabled(output);
  ui->pushButtonConvertAll->setEnabled(output);

  bool contains_tf = !ui->tableWidgetInput->findItems("/tf", Qt::MatchExactly).empty();
  ui->pushButtonFilterTF->setEnabled(ui->checkBoxFilterTF->isChecked() && contains_tf);
}

void RosbagEditor::on_pushButtonMove_pressed()
{
  QModelIndexList selected_input = ui->tableWidgetInput->selectionModel()->selectedRows();
  if (selected_input.count() == 0)
  {
    return;
  }

  for (int i = 0; i < selected_input.count(); i++)
  {
    QModelIndex index = selected_input.at(i);
    QTableWidgetItem *item = ui->tableWidgetInput->item(index.row(), 0);
    QString topic_name = item->text();

    if (ui->tableWidgetOutput->findItems(topic_name, Qt::MatchExactly).isEmpty())
    {
      int row = ui->tableWidgetOutput->rowCount();
      ui->tableWidgetOutput->setRowCount(row + 1);
      ui->tableWidgetOutput->setItem(row, 0, new QTableWidgetItem(topic_name));
      QLineEdit *topic_editor = new QLineEdit(ui->tableWidgetOutput);
      ui->tableWidgetOutput->setCellWidget(row, 1, topic_editor);
    }
  }

  ui->tableWidgetInput->selectionModel()->clearSelection();

  ui->pushButtonSave->setEnabled(ui->tableWidgetOutput->rowCount());
  ui->pushButtonConvert->setEnabled(ui->tableWidgetOutput->rowCount());
  ui->pushButtonConvertAll->setEnabled(ui->tableWidgetOutput->rowCount());

  ui->dateTimeOutputBegin->setDateTimeRange(ui->dateTimeInputBegin->dateTime(),
                                            ui->dateTimeInputEnd->dateTime());
  ui->dateTimeOutputBegin->setDateTime(ui->dateTimeInputBegin->dateTime());

  ui->dateTimeOutputEnd->setDateTimeRange(ui->dateTimeInputBegin->dateTime(),
                                          ui->dateTimeInputEnd->dateTime());
  ui->dateTimeOutputEnd->setDateTime(ui->dateTimeInputEnd->dateTime());

  ui->dateTimeEditCurrent->setValue(ui->dateTimeInputBegin->dateTime().toMSecsSinceEpoch());

  changeEnabledWidgets();
}

void RosbagEditor::on_tableWidgetInput_itemSelectionChanged()
{
  QItemSelectionModel *select = ui->tableWidgetInput->selectionModel();
  ui->pushButtonMove->setEnabled(select->hasSelection());
}

void RosbagEditor::on_tableWidgetOutput_itemSelectionChanged()
{
  QItemSelectionModel *select = ui->tableWidgetOutput->selectionModel();
  ui->pushButtonRemove->setEnabled(select->hasSelection());
}

void RosbagEditor::on_pushButtonRemove_pressed()
{
  QModelIndexList indexes;
  while ((indexes = ui->tableWidgetOutput->selectionModel()->selectedIndexes()).size())
  {
    ui->tableWidgetOutput->model()->removeRow(indexes.first().row());
  }

  ui->tableWidgetOutput->sortItems(0);

  changeEnabledWidgets();
}

void RosbagEditor::processSlider()
{
  if (_bag.isOpen())
  {
    // rosbag::View bag_view ( _bag );

    int32_t pos = ui->horizontalSlider->value();

    //
    // std::cout<<pos<<" "<<ui->labelCount->text().toInt()<<" "<<(pos+0.01)/play_count_<<" "<<sec_begin_+(sec_end_-sec_begin_)*((pos+0.01)/play_count_)<<std::endl;
    QDateTime datetime_current = QDateTime::fromMSecsSinceEpoch((sec_begin_ + (sec_end_ - sec_begin_) * ((pos + 0.01) / play_count_)) * 1000.);

    // ui->dateTimeEditCurrent->setDateTime(datetime_current);
    // std::cout<<datetime_current.toTime_t()<<" "<<datetime_current.toMSecsSinceEpoch()<<std::endl;

    ui->dateTimeEditCurrent->setValue(datetime_current.toMSecsSinceEpoch());
  }
}

void RosbagEditor::on_pushButtonUpdateSlider_pressed()
{
  // double currenttime = ui->dateTimeEditCurrent->dateTime().toTime_t();
  double currenttime = ui->dateTimeEditCurrent->value() / 1000.;
  int32_t pos = (currenttime - sec_begin_) / (sec_end_ - sec_begin_) * play_count_;
  ui->horizontalSlider->setValue(pos);
}

void RosbagEditor::PerviewSlot(int row, int col)
{
  std::cout << row << " " << col << " " << ui->tableWidgetInput->item(row, 0)->text().toStdString() << " " << ui->tableWidgetInput->item(row, 1)->text().toStdString() << std::endl;

  QString datatype = ui->tableWidgetInput->item(row, 1)->text();
  if (datatype == "sensor_msgs/Image")
  {
    if (!img_viewer_)
    {
      img_viewer_.reset(new ImageViewer);
      img_viewer_->show();
    }
  }
  else if (datatype == "sensor_msgs/CompressedImage")
  {
    if (!img_viewer_)
    {
      img_viewer_.reset(new ImageViewer);
      img_viewer_->show();
    }
  }
}

void RosbagEditor::on_pushButtonPause_pressed()
{
  paused_ = true;
}

void RosbagEditor::on_pushButtonLidarCamCalib_pressed()
{
  QString config_path_ = QFileDialog::getOpenFileName(this,
                                                      tr("Open File"),
                                                      ".",
                                                      tr("Config JSON Files(*.json)"));

  if (config_path_.isEmpty())
  {
    return;
  }
  ui->lineEditLidarCamCalib->setText(config_path_);

  std::ifstream f(config_path_.toStdString());
  if (!f.good())
  {
    f.close();
    return;
  }

  try
  {
    f >> js_;
  }
  catch (nlohmann::json::parse_error &e)
  {
    std::cerr << e.what();
    f.close();
    return;
  }
  calibrator_.reset(new lqh::Calibrator(js_));

  auto &flt = js_["pc"]["filter"];

  // Eigen::Matrix3d K;
  // Eigen::Matrix<double, 5, 1> D;
  auto &JK = js_["cam"]["K"];
  auto &JD = js_["cam"]["D"];

  K_ = cv::Mat(3, 3, CV_32FC1);
  for (uint8_t i = 0; i < 9; i++)
  {
    K_.ptr<float>(i / 3)[i % 3] = JK[i / 3][i % 3];
  }
  flag_kd_ |= 0x01;

  D_ = cv::Mat(5, 1, CV_32FC1);
  for (uint8_t i = 0; i < 5; i++)
  {
    D_.ptr<float>(i)[0] = JD[i];
  }

  flag_kd_ |= 0x02;
}
void RosbagEditor::on_pushButtonNext_pressed()
{
}

void RosbagEditor::on_pushButtonSnapshot_pressed()
{
  snapshot_ = true;
}

void RosbagEditor::on_pushButtonConvert_pressed()
{
  QString outputdir = QFileDialog::getExistingDirectory(this, tr("Save Directory"),
                                                        QDir::homePath(),
                                                        QFileDialog::ShowDirsOnly);
  if (outputdir.isEmpty())
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::vector<std::string> input_topics;
  std::map<std::string, std::string> topis_renamed, topic_type;
  int imgtopic_cnt = 0;

  paused_ = false;

  for (int row = 0; row < ui->tableWidgetInput->rowCount(); ++row)
  {
    std::string namet = ui->tableWidgetInput->item(row, 0)->text().toStdString();
    std::string typet = ui->tableWidgetInput->item(row, 1)->text().toStdString();
    topic_type.insert(std::make_pair(namet, typet));
  }

  for (int row = 0; row < ui->tableWidgetOutput->rowCount(); ++row)
  {
    std::string name = ui->tableWidgetOutput->item(row, 0)->text().toStdString();
    QLineEdit *line_edit = qobject_cast<QLineEdit *>(ui->tableWidgetOutput->cellWidget(row, 1));
    std::string renamed = line_edit->text().toStdString();
    input_topics.push_back(name);
    std::string selecttype = topic_type.find(name)->second;
    if (selecttype == "sensor_msgs/Image" || selecttype == "sensor_msgs/CompressedImage")
    {
      imgtopic_cnt++;
    }
    if (renamed.empty())
    {
      topis_renamed.insert(std::make_pair(name, name));
    }
    else
    {
      topis_renamed.insert(std::make_pair(name, renamed));
    }
  }

  // double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->dateTime().toMSecsSinceEpoch()));
  // double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));
  double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->value()));
  double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));

  rosbag::View bag_view(_bag, rosbag::TopicQuery(input_topics),
                        ros::Time(begin_time), ros::Time(end_time));
  rosbag::View bag_view_t(_bag, rosbag::TopicQuery(input_topics));

  play_count_ = bag_view_t.size();
  ui->horizontalSlider->setMaximum(play_count_ - 1);

  bool is_flip, is_mirror;
  is_flip = ui->checkBoxFlip->isChecked();
  is_mirror = ui->checkBoxMirror->isChecked();

  boost::shared_ptr<velodyne_rawdata::RawData> data_ = boost::make_shared<velodyne_rawdata::RawData>();
  QString calibfile = ui->lineEditVeloCalib->text();
  if (calibfile.isEmpty())
  {
    int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file not found, convert packets need calib, continue anyway?",
                                    QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
    if (ret == QMessageBox::Close)
    {
      return;
    }
    // calibfile="/home/nn1/catkin2/HDL-32.yaml";
  }
  else
  {
    int errret = data_->setupOffline(calibfile.toStdString(), 130, 0.4);
    if (errret < 0)
    {
      int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file load error, continue anyway?",
                                      QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
      if (ret == QMessageBox::Close)
      {
        return;
      }
    }
    data_->setParameters(0.4, 130.0, 0, 2 * 3.141592653);
  }

  bool sync_lock = false;
  ros::Time::init();
  ros::Time imgtime1_;
  ros::Time imgtime2_;
  ros::Time pctime1_;
  ros::Time pctime2_;

  int skipnum = int(ui->spinBoxGrid->value() / 100);
  int velocnt = 0;

  std::map<std::string, std::vector<std::pair<ros::Time, cv::Mat>>> msgcache;                    //多个相机图片
  std::vector<std::pair<ros::Time, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> pccache; //单个点云
  ros::Time pctime;
  std::vector<ros::Time> imgtimevc;
  for (const rosbag::MessageInstance &msg : bag_view)
  {

    ui->horizontalSlider->setValue(ui->horizontalSlider->value() + 1); //时间轴修改

    // processSlider();
    const auto &name = topis_renamed.find(msg.getTopic())->second;
    // std::cout << msg.getTopic() << " " << msg.getDataType() << std::endl;

    // std::map<std::string,std::vector<std::pair<std::string,std::string>>> msgcache;
    if (msg.getDataType() == "sensor_msgs/Image" || msg.getDataType() == "sensor_msgs/CompressedImage")
    {
      cv::Mat current_image;
      if (msg.getDataType() == "sensor_msgs/Image")
      {
        sensor_msgs::Image::Ptr in_image_msg = msg.instantiate<sensor_msgs::Image>();
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
        current_image = cv_image->image;
        imgtime1_ = in_image_msg->header.stamp;
      }
      else if (msg.getDataType() == "sensor_msgs/CompressedImage")
      {

        sensor_msgs::CompressedImage::Ptr in_image_msg = msg.instantiate<sensor_msgs::CompressedImage>();
        QString imgtype = QString::fromStdString(name).split("/").at(2);
        if (imgtype == "image_h264")
        {

          AVPacket packet;
          av_init_packet(&packet);
          packet.data = const_cast<uint8_t *>(in_image_msg->data.data());
          packet.size = in_image_msg->data.size();
          packet.pts = AV_NOPTS_VALUE;
          packet.dts = AV_NOPTS_VALUE;

          AVFrame frame;
          memset(&frame, 0, sizeof(frame));

          int gotPicture;

          if (avcodec_decode_video2(g_codec, &frame, &gotPicture, &packet) < 0)
          {
            continue;
          }

          if (gotPicture)
          {

            g_sws = sws_getCachedContext(
                g_sws,
                frame.width, frame.height, AV_PIX_FMT_YUV420P,
                frame.width, frame.height, AV_PIX_FMT_RGB24,
                0, 0, 0, 0);

            sensor_msgs::ImagePtr img(new sensor_msgs::Image);

            img->encoding = "rgb8";
            img->data.resize(frame.width * frame.height * 3);
            img->step = frame.width * 3;
            img->width = frame.width;
            img->height = frame.height;
            img->header.frame_id = "cam";
            img->header.stamp = ros::Time::now(); // FIXME

            uint8_t *destData[1] = {img->data.data()};
            int linesize[1] = {(int)img->step};

            sws_scale(g_sws, frame.data, frame.linesize, 0, frame.height,
                      destData, linesize);

            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            current_image = cv_image->image;
          }
        }
        else
        {
          current_image = cv::imdecode(cv::Mat(in_image_msg->data), 1);
        }
        imgtime1_ = in_image_msg->header.stamp;
      }

      //将每张图添加到map
      auto iter = msgcache.find(name);
      if (iter == msgcache.end())
      {
        std::vector<std::pair<ros::Time, cv::Mat>> vectertmp;
        vectertmp.push_back(std::make_pair(imgtime1_, current_image));
        // std::cout<<"f"<<vectertmp.size()<<std::endl;
        msgcache.insert(std::make_pair(name, vectertmp));
      }
      else
      {
        std::vector<std::pair<ros::Time, cv::Mat>> vectertmp;
        vectertmp.swap(iter->second);
        vectertmp.push_back(std::make_pair(imgtime1_, current_image));
        // std::cout<<"s"<<vectertmp.size()<<" "<<in_image_msg->header.stamp<<std::endl;
        msgcache[name].swap(vectertmp);
      }

      std::vector<ros::Time>::iterator imgtime;
      std::vector<std::pair<ros::Time, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> pcvaluetmp;

      if (imgtimevc.size() > 0)
      {
        imgtime = std::max_element(imgtimevc.begin(), imgtimevc.end());
        //将时间小于图像时间的点云删除
        if (sync_lock == false) //没锁就删
        {
          // std::cout << "pc:" << pccache.size() << " " << pccache[0].first << " img2 " << msgcache2.size() << std::endl;
          for (auto itemp = pccache.begin(); itemp != pccache.end(); itemp++)
          {
            std::cout << "pc keep:" << itemp->first << " " << (itemp->first - *imgtime).toNSec() << " " << (itemp->first - *imgtime).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() << std::endl;

            if ((itemp->first - *imgtime).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() > 0)
            {
              // value.erase(itemv);
              pcvaluetmp.push_back(*itemp);
            }
            // else
            // {

            // }
            pctime = pccache.begin()->first;
          }
          if (pcvaluetmp.size() > 0)
          {
            pctime = pcvaluetmp.begin()->first;
            pccache.assign(pcvaluetmp.begin(), pcvaluetmp.end());
            sync_lock = true; //删了就加锁
          }
        }
      }

      // std::cout << "pccache.size() " << pccache.size() << " msgcache.size() " << msgcache.size() << std::endl;
      std::map<std::string, std::vector<std::pair<ros::Time, cv::Mat>>> msgcache2;
      if (pccache.size() > 0)
      {
        imgtimevc.empty();
        for (auto item = msgcache.begin(); item != msgcache.end(); item++)
        {
          std::vector<std::pair<ros::Time, cv::Mat>> valuetmp;
          auto key = item->first;
          auto value = item->second;
          // auto lastitemv=value.begin();
          for (auto itemv = value.begin(); itemv != value.end(); itemv++)
          {
            //对于图片时间小于点云删除
            // if(abs((pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt())<70){
            //   std::cout << "img keep:" << pctime << " " << (pctime - itemv->first).toNSec() << " " << (pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() << std::endl;
            // }
            // std::cout<<(pctime - lastitemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() <<" "<<(pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() <<std::endl;
            if ((pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() < 0)
            {
              // value.erase(itemv);
              valuetmp.push_back(*itemv);
            }
            // lastitemv=itemv;
          }
          //每个字典的元素长度
          // std::cout <<"-key: "<<key<< " size: " << value.size() << std::endl;

          if (valuetmp.size() > 0)
          {
            // std::cout << "img:" << key << " " << valuetmp.size() << " " << valuetmp[0].first << std::endl;
            // sync_lock = true;
            //添加到新的字典
            msgcache2.insert(std::make_pair(key, valuetmp));
            imgtimevc.push_back(valuetmp[0].first);
          }
        }
      }
      //交换图像字典数据
      msgcache.swap(msgcache2);

      //没锁就不写，没达到预定数量图片不写
      std::cout << "msgcache size: " << msgcache.size() << " imgtopic_cnt:" << imgtopic_cnt << std::endl;
      if (sync_lock == false || msgcache.size() != imgtopic_cnt)
      {
        continue;
      }
      //显示或写入磁盘

      for (auto item = msgcache.begin(); item != msgcache.end(); item++)
      {
        // std::cout << "-img cnt:" << item->second.size() << " pc cnt:" << pccache.size() << " sync:" << sync_lock << std::endl;
        if (item->second.size() == 0 || pccache.size() == 0)
        {
          continue;
        }

        cv::Mat current_image_t;
        item->second[0].second.copyTo(current_image_t);
        imgtime1_ = ros::Time::now();
        imgtime2_ = item->second[0].first;
        std::cout << "-pc time:" << pctime << " img time:" << imgtime2_ << " delay:" << pctime - imgtime2_ << std::endl;
        //对比点云图像时间

        //翻转镜像
        if (is_flip)
        {
          cv::flip(current_image_t, current_image_t, 0);
        }
        if (is_mirror)
        {
          cv::flip(current_image_t, current_image_t, 1);
        }
        // std::shared_ptr<cv::Mat> img_marked = std::make_shared<cv::Mat>();

        // //畸变校正
        // if (flag_kd_ == 0)
        // {
        //   current_image_t.copyTo(*img_marked);
        // }
        // else
        // {
        //   cv::undistort(current_image_t, *img_marked, K_, D_);
        // }
        // // continue;

        //写入
        std::stringstream ss1;
        ss1 << outputdir.toStdString() << "/" << QString::fromStdString(item->first).split("/").at(1).toStdString() << "/";
        QString fullPath1 = QString::fromStdString(ss1.str());
        QDir imgdir1(fullPath1);
        if (!imgdir1.exists())
        {
          bool ok = imgdir1.mkpath(fullPath1); //创建多级目录
        }
        ss1 << imgtime2_.toNSec() / 1000000 << "_";
        ss1 << ((int)imgtime2_.toNSec() - (int)pccache.begin()->first.toNSec()) / 1000000 << ".jpg";
        std::string file_path1 = ss1.str();
        cv::imwrite(file_path1, current_image_t);
        std::cout << file_path1 << std::endl;

        std::stringstream ss2;
        ss2 << outputdir.toStdString() << "/pointcloud/";
        QString fullPath2 = QString::fromStdString(ss2.str());
        QDir imgdir2(fullPath2);
        if (!imgdir2.exists())
        {
          bool ok = imgdir2.mkpath(fullPath2); //创建多级目录
        }
        ss2 << imgtime2_.toNSec() / 1000000 << "_";
        ss2 << ((int)imgtime2_.toNSec() - (int)pccache.begin()->first.toNSec()) / 1000000 << ".pcd";
        std::string file_path2 = ss2.str();
        std::cout << file_path2 << std::endl;
        if (!QFile::exists(QString::fromStdString(file_path2)))
        {
          // save PCD file as well
          pcl::copyPointCloud(*pccache.begin()->second, *pc);
          pcl::io::savePCDFileASCII(file_path2, *pc);
        }

        sync_lock = false;
        cv::waitKey(1);
      }
    }
    else if (msg.getDataType() == "velodyne_msgs/VelodyneScan")
    {
      velocnt++;
      if (velocnt == skipnum)
      {
        velocnt = 0;
      }
      else
      {
        continue;
      }
      velodyne_msgs::VelodyneScan::Ptr in_sensor_cloud = msg.instantiate<velodyne_msgs::VelodyneScan>();
      // msgcache.insert(std::make_pair(name,std::make_pair(in_sensor_cloud->header.stamp,*msg)));

      velodyne_rawdata::VPointCloud::Ptr
          outMsg(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(in_sensor_cloud->header).stamp;
      outMsg->header.frame_id = in_sensor_cloud->header.frame_id;
      outMsg->height = 1;

      pctime1_ = ros::Time::now();
      pctime2_ = in_sensor_cloud->header.stamp;

      std::cout << in_sensor_cloud->header.stamp << " " << msg.getTime() << std::endl;

      // process each packet provided by the driver
      for (size_t i = 0; i < in_sensor_cloud->packets.size(); ++i)
      {
        data_->unpack(in_sensor_cloud->packets[i], *outMsg);
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr pctmp(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud(*outMsg, *pctmp);
      pccache.push_back(std::make_pair(in_sensor_cloud->header.stamp, pctmp));
    }
    else if (msg.getDataType() == "sensor_msgs/PointCloud2")
    {
      velocnt++;
      if (velocnt == skipnum)
      {
        velocnt = 0;
      }
      else
      {
        continue;
      }
      sensor_msgs::PointCloud2::Ptr in_sensor_cloud = msg.instantiate<sensor_msgs::PointCloud2>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*in_sensor_cloud, *velodyne_cloud_ptr);

      //save PCD file as well
      // pcl::io::savePCDFileASCII(file_path, *velodyne_cloud_ptr);
      pccache.push_back(std::make_pair(in_sensor_cloud->header.stamp, velodyne_cloud_ptr));
    }
    if (paused_)
    {
      break;
    }
    // break;
  }
  int ret = QMessageBox::question(this, "Done", "Convert succesfully created. Do you want to close the application?",
                                  QMessageBox::Cancel | QMessageBox::Close, QMessageBox::Close);
  if (ret == QMessageBox::Close)
  {
    this->close();
  }
}

void RosbagEditor::on_pushButtonConvertAll_pressed()
{
  QString outputdir = QFileDialog::getExistingDirectory(this, tr("Save Directory"),
                                                        QDir::homePath(),
                                                        QFileDialog::ShowDirsOnly);
  if (outputdir.isEmpty())
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::vector<std::string> input_topics;
  std::map<std::string, std::string> topis_renamed, topic_type;
  int imgtopic_cnt = 0;

  paused_ = false;

  for (int row = 0; row < ui->tableWidgetInput->rowCount(); ++row)
  {
    std::string namet = ui->tableWidgetInput->item(row, 0)->text().toStdString();
    std::string typet = ui->tableWidgetInput->item(row, 1)->text().toStdString();
    topic_type.insert(std::make_pair(namet, typet));
  }

  for (int row = 0; row < ui->tableWidgetOutput->rowCount(); ++row)
  {
    std::string name = ui->tableWidgetOutput->item(row, 0)->text().toStdString();
    QLineEdit *line_edit = qobject_cast<QLineEdit *>(ui->tableWidgetOutput->cellWidget(row, 1));
    std::string renamed = line_edit->text().toStdString();
    input_topics.push_back(name);
    std::string selecttype = topic_type.find(name)->second;
    if (selecttype == "sensor_msgs/Image" || selecttype == "sensor_msgs/CompressedImage")
    {
      imgtopic_cnt++;
    }
    if (renamed.empty())
    {
      topis_renamed.insert(std::make_pair(name, name));
    }
    else
    {
      topis_renamed.insert(std::make_pair(name, renamed));
    }
  }

  // double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->dateTime().toMSecsSinceEpoch()));
  // double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));
  double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->value()));
  double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));

  rosbag::View bag_view(_bag, rosbag::TopicQuery(input_topics),
                        ros::Time(begin_time), ros::Time(end_time));
  rosbag::View bag_view_t(_bag, rosbag::TopicQuery(input_topics));

  play_count_ = bag_view_t.size();
  ui->horizontalSlider->setMaximum(play_count_ - 1);

  bool is_flip, is_mirror;
  is_flip = ui->checkBoxFlip->isChecked();
  is_mirror = ui->checkBoxMirror->isChecked();

  boost::shared_ptr<velodyne_rawdata::RawData> data_ = boost::make_shared<velodyne_rawdata::RawData>();
  QString calibfile = ui->lineEditVeloCalib->text();
  if (calibfile.isEmpty())
  {
    int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file not found, convert packets need calib, continue anyway?",
                                    QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
    if (ret == QMessageBox::Close)
    {
      return;
    }
    // calibfile="/home/nn1/catkin2/HDL-32.yaml";
  }
  else
  {
    int errret = data_->setupOffline(calibfile.toStdString(), 130, 0.4);
    if (errret < 0)
    {
      int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file load error, continue anyway?",
                                      QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
      if (ret == QMessageBox::Close)
      {
        return;
      }
    }
    data_->setParameters(0.4, 130.0, 0, 2 * 3.141592653);
  }

  bool sync_lock = false;
  ros::Time::init();
  ros::Time imgtime1_;
  ros::Time imgtime2_;
  ros::Time pctime1_;
  ros::Time pctime2_;

  std::map<std::string, std::vector<std::pair<ros::Time, cv::Mat>>> msgcache;                    //多个相机图片
  std::vector<std::pair<ros::Time, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> pccache; //单个点云
  ros::Time pctime;
  std::vector<ros::Time> imgtimevc;
  for (const rosbag::MessageInstance &msg : bag_view)
  {

    ui->horizontalSlider->setValue(ui->horizontalSlider->value() + 1); //时间轴修改

    // processSlider();
    const auto &name = topis_renamed.find(msg.getTopic())->second;
    // std::cout << msg.getTopic() << " " << msg.getDataType() << std::endl;

    // std::map<std::string,std::vector<std::pair<std::string,std::string>>> msgcache;
    if (msg.getDataType() == "sensor_msgs/Image" || msg.getDataType() == "sensor_msgs/CompressedImage")
    {
      cv::Mat current_image;
      if (msg.getDataType() == "sensor_msgs/Image")
      {
        sensor_msgs::Image::Ptr in_image_msg = msg.instantiate<sensor_msgs::Image>();
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
        current_image = cv_image->image;
        imgtime1_ = in_image_msg->header.stamp;
      }
      else if (msg.getDataType() == "sensor_msgs/CompressedImage")
      {

        sensor_msgs::CompressedImage::Ptr in_image_msg = msg.instantiate<sensor_msgs::CompressedImage>();
        QString imgtype = QString::fromStdString(name).split("/").at(2);
        if (imgtype == "image_h264")
        {

          AVPacket packet;
          av_init_packet(&packet);
          packet.data = const_cast<uint8_t *>(in_image_msg->data.data());
          packet.size = in_image_msg->data.size();
          packet.pts = AV_NOPTS_VALUE;
          packet.dts = AV_NOPTS_VALUE;

          AVFrame frame;
          memset(&frame, 0, sizeof(frame));

          int gotPicture;

          if (avcodec_decode_video2(g_codec, &frame, &gotPicture, &packet) < 0)
          {
            continue;
          }

          if (gotPicture)
          {

            g_sws = sws_getCachedContext(
                g_sws,
                frame.width, frame.height, AV_PIX_FMT_YUV420P,
                frame.width, frame.height, AV_PIX_FMT_RGB24,
                0, 0, 0, 0);

            sensor_msgs::ImagePtr img(new sensor_msgs::Image);

            img->encoding = "rgb8";
            img->data.resize(frame.width * frame.height * 3);
            img->step = frame.width * 3;
            img->width = frame.width;
            img->height = frame.height;
            img->header.frame_id = "cam";
            img->header.stamp = ros::Time::now(); // FIXME

            uint8_t *destData[1] = {img->data.data()};
            int linesize[1] = {(int)img->step};

            sws_scale(g_sws, frame.data, frame.linesize, 0, frame.height,
                      destData, linesize);

            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            current_image = cv_image->image;
          }
        }
        else
        {
          current_image = cv::imdecode(cv::Mat(in_image_msg->data), 1);
        }

        imgtime1_ = in_image_msg->header.stamp;
      }
      cv::Mat current_image_t;
      current_image.copyTo(current_image_t);
      //翻转镜像
      if (is_flip)
      {
        cv::flip(current_image_t, current_image_t, 0);
      }
      if (is_mirror)
      {
        cv::flip(current_image_t, current_image_t, 1);
      }

      std::stringstream ss1;
      ss1 << outputdir.toStdString() << "/" << QString::fromStdString(name).split("/").at(1).toStdString() << "/";
      QString fullPath1 = QString::fromStdString(ss1.str());
      QDir imgdir1(fullPath1);
      if (!imgdir1.exists())
      {
        bool ok = imgdir1.mkpath(fullPath1); //创建多级目录
      }
      ss1 << imgtime1_.toNSec() / 1000000 << ".jpg";
      std::string file_path1 = ss1.str();
      cv::imwrite(file_path1, current_image_t);
      std::cout << file_path1 << std::endl;

      cv::waitKey(1);
    }
    else if (msg.getDataType() == "velodyne_msgs/VelodyneScan")
    {

      velodyne_msgs::VelodyneScan::Ptr in_sensor_cloud = msg.instantiate<velodyne_msgs::VelodyneScan>();
      // msgcache.insert(std::make_pair(name,std::make_pair(in_sensor_cloud->header.stamp,*msg)));

      velodyne_rawdata::VPointCloud::Ptr
          outMsg(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(in_sensor_cloud->header).stamp;
      outMsg->header.frame_id = in_sensor_cloud->header.frame_id;
      outMsg->height = 1;

      pctime1_ = ros::Time::now();
      pctime2_ = in_sensor_cloud->header.stamp;

      std::cout << in_sensor_cloud->header.stamp << " " << msg.getTime() << std::endl;

      // process each packet provided by the driver
      for (size_t i = 0; i < in_sensor_cloud->packets.size(); ++i)
      {
        data_->unpack(in_sensor_cloud->packets[i], *outMsg);
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr pctmp(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud(*outMsg, *pctmp);
      std::stringstream ss2;
      ss2 << outputdir.toStdString() << "/pointcloud/";
      QString fullPath2 = QString::fromStdString(ss2.str());
      QDir imgdir2(fullPath2);
      if (!imgdir2.exists())
      {
        bool ok = imgdir2.mkpath(fullPath2); //创建多级目录
      }
      ss2 << outMsg->header.stamp / 1000000 << ".pcd";
      std::string file_path2 = ss2.str();
      std::cout << file_path2 << std::endl;
      if (!QFile::exists(QString::fromStdString(file_path2)))
      {
        // save PCD file as well
        pcl::io::savePCDFileASCII(file_path2, *pctmp);
      }
    }
    else if (msg.getDataType() == "sensor_msgs/PointCloud2")
    {

      sensor_msgs::PointCloud2::Ptr in_sensor_cloud = msg.instantiate<sensor_msgs::PointCloud2>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*in_sensor_cloud, *velodyne_cloud_ptr);

      //save PCD file as well
      // pcl::io::savePCDFileASCII(file_path, *velodyne_cloud_ptr);
      std::stringstream ss2;
      ss2 << outputdir.toStdString() << "/pointcloud/";
      QString fullPath2 = QString::fromStdString(ss2.str());
      QDir imgdir2(fullPath2);
      if (!imgdir2.exists())
      {
        bool ok = imgdir2.mkpath(fullPath2); //创建多级目录
      }
      ss2 << in_sensor_cloud->header.stamp.toNSec() / 1000000 << ".pcd";
      std::string file_path2 = ss2.str();
      std::cout << file_path2 << std::endl;
      if (!QFile::exists(QString::fromStdString(file_path2)))
      {
        // save PCD file as well
        pcl::io::savePCDFileASCII(file_path2, *velodyne_cloud_ptr);
      }
    }
    if (paused_)
    {
      break;
    }
    // break;
  }
  int ret = QMessageBox::question(this, "Done", "Convert succesfully created. Do you want to close the application?",
                                  QMessageBox::Cancel | QMessageBox::Close, QMessageBox::Close);
  if (ret == QMessageBox::Close)
  {
    this->close();
  }
}
void RosbagEditor::on_pushButtonPlay_pressed()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::vector<std::string> input_topics;
  std::map<std::string, std::string> topis_renamed, topic_type;
  int imgtopic_cnt = 0;

  paused_ = false;
  if (!img_viewer_)
  {
    img_viewer_.reset(new ImageViewer);
    img_viewer_->show();
  }
  if (!pc_viewer_)
  {
    pc_viewer_.reset(new PointcloudViewer);
    pc_viewer_->show();
  }
  for (int row = 0; row < ui->tableWidgetInput->rowCount(); ++row)
  {
    std::string namet = ui->tableWidgetInput->item(row, 0)->text().toStdString();
    std::string typet = ui->tableWidgetInput->item(row, 1)->text().toStdString();
    topic_type.insert(std::make_pair(namet, typet));
  }

  for (int row = 0; row < ui->tableWidgetOutput->rowCount(); ++row)
  {
    std::string name = ui->tableWidgetOutput->item(row, 0)->text().toStdString();
    QLineEdit *line_edit = qobject_cast<QLineEdit *>(ui->tableWidgetOutput->cellWidget(row, 1));
    std::string renamed = line_edit->text().toStdString();
    input_topics.push_back(name);
    std::string selecttype = topic_type.find(name)->second;
    if (selecttype == "sensor_msgs/Image" || selecttype == "sensor_msgs/CompressedImage")
    {
      imgtopic_cnt++;
    }
    if (renamed.empty())
    {
      topis_renamed.insert(std::make_pair(name, name));
    }
    else
    {
      topis_renamed.insert(std::make_pair(name, renamed));
    }
  }

  // double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->dateTime().toMSecsSinceEpoch()));
  // double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));
  double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->value()));
  double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));

  rosbag::View bag_view(_bag, rosbag::TopicQuery(input_topics),
                        ros::Time(begin_time), ros::Time(end_time));
  rosbag::View bag_view_t(_bag, rosbag::TopicQuery(input_topics));

  play_count_ = bag_view_t.size();
  ui->horizontalSlider->setMaximum(play_count_ - 1);

  bool is_flip, is_mirror, is_fisheye;
  is_flip = ui->checkBoxFlip->isChecked();
  is_mirror = ui->checkBoxMirror->isChecked();
  is_fisheye = ui->checkBoxFisheye->isChecked();

  boost::shared_ptr<velodyne_rawdata::RawData> data_ = boost::make_shared<velodyne_rawdata::RawData>();
  QString calibfile = ui->lineEditVeloCalib->text();
  if (calibfile.isEmpty())
  {
    int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file not found, convert packets need calib, continue anyway?",
                                    QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
    if (ret == QMessageBox::Close)
    {
      return;
    }
    // calibfile="/home/nn1/catkin2/HDL-32.yaml";
  }
  else
  {
    int errret = data_->setupOffline(calibfile.toStdString(), 130, 0.4);
    if (errret < 0)
    {
      int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file load error, continue anyway?",
                                      QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
      if (ret == QMessageBox::Close)
      {
        return;
      }
    }
    data_->setParameters(0.4, 130.0, 0, 2 * 3.141592653);
  }

  bool sync_lock = false;
  ros::Time::init();
  ros::Time imgtime1_;
  ros::Time imgtime2_;
  ros::Time pctime1_;
  ros::Time pctime2_;

  int skipnum = int(ui->spinBoxGrid->value() / 100);
  if (skipnum > 4)
  {
    skipnum -= 2;
  }
  int velocnt = 0;
  QString outputdir;

  std::map<std::string, std::vector<std::pair<ros::Time, cv::Mat>>> msgcache;                    //多个相机图片
  std::vector<std::pair<ros::Time, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> pccache; //单个点云
  ros::Time pctime;
  std::vector<ros::Time> imgtimevc;
  // for (const rosbag::MessageInstance &msg : bag_view)
  for (auto msgit = bag_view.begin(); msgit != bag_view.end(); msgit++)
  {
    ui->horizontalSlider->setValue(ui->horizontalSlider->value() + 1); //时间轴修改
    if (velocnt != skipnum)                                            //跳过一些帧
    {
      if (msgit->getDataType() == "velodyne_msgs/VelodyneScan" || msgit->getDataType() == "sensor_msgs/PointCloud2")
      {
        velocnt++;
      }
      continue;
    }

    // std::cout<<msgit->getTime()<<std::endl;
    // continue;

    // processSlider();
    const auto &name = topis_renamed.find(msgit->getTopic())->second;
    // std::cout << msgit->getTopic() << " " << msgit->getDataType() << std::endl;

    // std::map<std::string,std::vector<std::pair<std::string,std::string>>> msgcache;
    if (msgit->getDataType() == "sensor_msgs/Image" || msgit->getDataType() == "sensor_msgs/CompressedImage")
    {
      cv::Mat current_image;
      if (msgit->getDataType() == "sensor_msgs/Image")
      {
        sensor_msgs::Image::Ptr in_image_msg = msgit->instantiate<sensor_msgs::Image>();
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
        current_image = cv_image->image;
        imgtime1_ = in_image_msg->header.stamp;
      }
      else if (msgit->getDataType() == "sensor_msgs/CompressedImage")
      {

        sensor_msgs::CompressedImage::Ptr in_image_msg = msgit->instantiate<sensor_msgs::CompressedImage>();
        QString imgtype = QString::fromStdString(name).split("/").at(2);
        if (imgtype == "image_h264")
        {

          AVPacket packet;
          av_init_packet(&packet);
          packet.data = const_cast<uint8_t *>(in_image_msg->data.data());
          packet.size = in_image_msg->data.size();
          packet.pts = AV_NOPTS_VALUE;
          packet.dts = AV_NOPTS_VALUE;

          AVFrame frame;
          memset(&frame, 0, sizeof(frame));

          int gotPicture;

          if (avcodec_decode_video2(g_codec, &frame, &gotPicture, &packet) < 0)
          {
            continue;
          }

          if (gotPicture)
          {

            g_sws = sws_getCachedContext(
                g_sws,
                frame.width, frame.height, AV_PIX_FMT_YUV420P,
                frame.width, frame.height, AV_PIX_FMT_RGB24,
                0, 0, 0, 0);

            sensor_msgs::ImagePtr img(new sensor_msgs::Image);

            img->encoding = "rgb8";
            img->data.resize(frame.width * frame.height * 3);
            img->step = frame.width * 3;
            img->width = frame.width;
            img->height = frame.height;
            img->header.frame_id = "cam";
            img->header.stamp = ros::Time::now(); // FIXME

            uint8_t *destData[1] = {img->data.data()};
            int linesize[1] = {(int)img->step};

            sws_scale(g_sws, frame.data, frame.linesize, 0, frame.height,
                      destData, linesize);

            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            current_image = cv_image->image;
          }
        }
        else
        {
          current_image = cv::imdecode(cv::Mat(in_image_msg->data), 1);
        }
        imgtime1_ = in_image_msg->header.stamp;
        // std::cout<<msgit->getTime()<<" "<<imgtime1_<<std::endl;
      }

      //将每张图添加到map
      auto iter = msgcache.find(name);
      if (iter == msgcache.end())
      {
        std::vector<std::pair<ros::Time, cv::Mat>> vectertmp;
        vectertmp.push_back(std::make_pair(imgtime1_, current_image));
        // std::cout<<"f"<<vectertmp.size()<<std::endl;
        msgcache.insert(std::make_pair(name, vectertmp));
      }
      else
      {
        std::vector<std::pair<ros::Time, cv::Mat>> vectertmp;
        vectertmp.swap(iter->second);
        vectertmp.push_back(std::make_pair(imgtime1_, current_image));
        // std::cout<<"s"<<vectertmp.size()<<" "<<in_image_msg->header.stamp<<std::endl;
        msgcache[name].swap(vectertmp);
      }

      std::vector<ros::Time>::iterator imgtime;
      std::vector<std::pair<ros::Time, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> pcvaluetmp;

      if (imgtimevc.size() > 0)
      {
        imgtime = std::max_element(imgtimevc.begin(), imgtimevc.end());
        //将时间小于图像时间的点云删除
        if (sync_lock == false) //没锁就删
        {
          // std::cout << "pc:" << pccache.size() << " " << pccache[0].first << " img2 " << msgcache2.size() << std::endl;
          for (auto itemp = pccache.begin(); itemp != pccache.end(); itemp++)
          {
            // std::cout << "pc keep:" << itemp->first << " " << (itemp->first - *imgtime).toNSec() << " " << (itemp->first - *imgtime).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() << std::endl;

            if ((itemp->first - *imgtime).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() > 0)
            {
              // value.erase(itemv);
              pcvaluetmp.push_back(*itemp);
            }
            // else
            // {

            // }
            pctime = pccache.begin()->first;
          }
          if (pcvaluetmp.size() > 0)
          {
            pctime = pcvaluetmp.begin()->first;
            pccache.assign(pcvaluetmp.begin(), pcvaluetmp.end());
            sync_lock = true; //删了就加锁
          }
        }
      }

      std::cout << "pccache.size() " << pccache.size() << " msgcache.size() " << msgcache.size() << std::endl;
      std::map<std::string, std::vector<std::pair<ros::Time, cv::Mat>>> msgcache2;
      if (pccache.size() > 0)
      {

        for (auto item = msgcache.begin(); item != msgcache.end(); item++)
        {
          std::vector<std::pair<ros::Time, cv::Mat>> valuetmp;
          auto key = item->first;
          auto value = item->second;

          for (auto itemv = value.begin(); itemv != value.end(); itemv++)
          {
            //对于图片时间小于点云删除
            // std::cout << "img keep:" << pctime << " " << (pctime - itemv->first).toNSec() << " " << (pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() << std::endl;
            if ((pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() < 0)
            {
              // value.erase(itemv);
              valuetmp.push_back(*itemv);
            }
          }
          //每个字典的元素长度
          // std::cout <<"-key: "<<key<< " size: " << value.size() << std::endl;

          if (valuetmp.size() > 0)
          {
            // std::cout << "img:" << key << " " << valuetmp.size() << " " << valuetmp[0].first << std::endl;
            // sync_lock = true;
            //添加到新的字典
            msgcache2.insert(std::make_pair(key, valuetmp));
            imgtimevc.push_back(valuetmp[0].first);
          }
        }
        //交换图像字典数据
        msgcache.swap(msgcache2);
      }

      //没锁就不写，没达到预定数量图片不写
      std::cout << "msgcache size: " << msgcache.size() << " imgtopic_cnt:" << imgtopic_cnt << " lock:" << sync_lock << std::endl;
      if (sync_lock == false || msgcache.size() != imgtopic_cnt)
      {
        continue;
      }
      //显示或写入磁盘

      for (auto item = msgcache.begin(); item != msgcache.end(); item++)
      {

        // std::cout << "-img cnt:" << item->second.size() << " pc cnt:" << pccache.size() << " sync:" << sync_lock << std::endl;
        if (item->second.size() == 0 || pccache.size() == 0 || item->first != input_topics[0]) //只投影第一个图片topic
        {
          continue;
        }

        cv::Mat current_image_t;
        item->second[0].second.copyTo(current_image_t);
        imgtime1_ = ros::Time::now();
        imgtime2_ = item->second[0].first;
        std::cout << "-pc time:" << pctime << " img time:" << imgtime2_ << " delay:" << pctime - imgtime2_ << std::endl;
        //对比点云图像时间

        //翻转镜像
        if (is_flip)
        {
          cv::flip(current_image_t, current_image_t, 0);
        }
        if (is_mirror)
        {
          cv::flip(current_image_t, current_image_t, 1);
        }

        std::shared_ptr<cv::Mat> img_marked = std::make_shared<cv::Mat>();
        //畸变校正
        if (flag_kd_ == 0)
        {
          current_image_t.copyTo(*img_marked);
        }
        else
        {
          if (is_fisheye)
          {
            cv::Mat D;
            D_.copyTo(D);
            D.pop_back();
            //        std::cout<<K_<<D<<std::endl;
            cv::Matx33d newK = K_;
            cv::fisheye::undistortImage(current_image_t, *img_marked, K_, D, newK);
          }
          else
          {
            cv::undistort(current_image_t, *img_marked, K_, D_);
          }
        }
        // continue;

        //投影
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcc(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*pccache.begin()->second, *pcc);
        for (auto &p : pcc->points)
        {
          p.rgba = 0xffffffff;
        }
        std::shared_ptr<cv::Mat> img_mark = std::make_shared<cv::Mat>();
        img_marked->copyTo(*img_mark);

        calibrator_->Project(*pcc, *img_mark);

        img_viewer_->showImage(img_mark);

        pc_viewer_->showPointcloud(pcc);
        sync_lock = false;
      }
      if (snapshot_)
      { //截图

        if (outputdir == "")
        {
          outputdir = QFileDialog::getExistingDirectory(this, tr("Save Directory"),
                                                        QDir::homePath(),
                                                        QFileDialog::ShowDirsOnly);
          if (outputdir.isEmpty())
          {
            continue;
          }
        }

        for (auto item = msgcache.begin(); item != msgcache.end(); item++)
        {
          // std::cout << "-img cnt:" << item->second.size() << " pc cnt:" << pccache.size() << " sync:" << sync_lock << std::endl;
          if (item->second.size() == 0 || pccache.size() == 0)
          {
            continue;
          }

          cv::Mat current_image_t;
          item->second[0].second.copyTo(current_image_t);
          imgtime1_ = ros::Time::now();
          imgtime2_ = item->second[0].first;
          std::cout << "-pc time:" << pctime << " img time:" << imgtime2_ << " delay:" << pctime - imgtime2_ << std::endl;
          //对比点云图像时间

          //翻转镜像
          if (is_flip)
          {
            cv::flip(current_image_t, current_image_t, 0);
          }
          if (is_mirror)
          {
            cv::flip(current_image_t, current_image_t, 1);
          }
          std::stringstream ss1;
          ss1 << outputdir.toStdString() << "/" << QString::fromStdString(item->first).split("/").at(1).toStdString() << "/";
          QString fullPath1 = QString::fromStdString(ss1.str());
          QDir imgdir1(fullPath1);
          if (!imgdir1.exists())
          {
            bool ok = imgdir1.mkpath(fullPath1); //创建多级目录
          }
          ss1 << pccache.begin()->first.toNSec() / 1000000 << ".jpg";
          std::string file_path1 = ss1.str();
          cv::imwrite(file_path1, current_image_t);
          std::cout << file_path1 << std::endl;

          std::stringstream ss2;
          ss2 << outputdir.toStdString() << "/pointcloud/";
          QString fullPath2 = QString::fromStdString(ss2.str());
          QDir imgdir2(fullPath2);
          if (!imgdir2.exists())
          {
            bool ok = imgdir2.mkpath(fullPath2); //创建多级目录
          }
          ss2 << pccache.begin()->first.toNSec() / 1000000 << ".pcd";
          std::string file_path2 = ss2.str();
          std::cout << file_path2 << std::endl;
          if (!QFile::exists(QString::fromStdString(file_path2)))
          {
            // save PCD file as well
            pcl::copyPointCloud(*pccache.begin()->second, *pc);
            pcl::io::savePCDFileASCII(file_path2, *pc);
          }
          snapshot_ = false;
        }
      }
      velocnt = 0;
      msgcache.empty();
      pccache.empty();
      imgtimevc.empty();
      cv::waitKey(1);
    }
    else if (msgit->getDataType() == "velodyne_msgs/VelodyneScan")
    {

      velodyne_msgs::VelodyneScan::Ptr in_sensor_cloud = msgit->instantiate<velodyne_msgs::VelodyneScan>();
      // msgcache.insert(std::make_pair(name,std::make_pair(in_sensor_cloud->header.stamp,*msg)));

      velodyne_rawdata::VPointCloud::Ptr
          outMsg(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(in_sensor_cloud->header).stamp;
      outMsg->header.frame_id = in_sensor_cloud->header.frame_id;
      outMsg->height = 1;

      pctime1_ = ros::Time::now();
      pctime2_ = in_sensor_cloud->header.stamp;

      std::cout << in_sensor_cloud->header.stamp << " " << msgit->getTime() << std::endl;

      // process each packet provided by the driver
      for (size_t i = 0; i < in_sensor_cloud->packets.size(); ++i)
      {
        data_->unpack(in_sensor_cloud->packets[i], *outMsg);
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr pctmp(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud(*outMsg, *pctmp);
      pccache.push_back(std::make_pair(in_sensor_cloud->header.stamp, pctmp));
    }
    else if (msgit->getDataType() == "sensor_msgs/PointCloud2")
    {

      sensor_msgs::PointCloud2::Ptr in_sensor_cloud = msgit->instantiate<sensor_msgs::PointCloud2>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*in_sensor_cloud, *velodyne_cloud_ptr);

      //save PCD file as well
      // pcl::io::savePCDFileASCII(file_path, *velodyne_cloud_ptr);
      pccache.push_back(std::make_pair(in_sensor_cloud->header.stamp, velodyne_cloud_ptr));
    }
    if (paused_)
    {
      break;
    }
    // break;
  }
}

void RosbagEditor::on_pushButtonSnapshotAll_pressed()
{
  QString outputdir = QFileDialog::getExistingDirectory(this, tr("Save Directory"),
                                                        QDir::homePath(),
                                                        QFileDialog::ShowDirsOnly);
  if (outputdir.isEmpty())
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::vector<std::string> input_topics;
  std::map<std::string, std::string> topis_renamed, topic_type;
  int imgtopic_cnt = 0;

  paused_ = false;

  for (int row = 0; row < ui->tableWidgetInput->rowCount(); ++row)
  {
    std::string namet = ui->tableWidgetInput->item(row, 0)->text().toStdString();
    std::string typet = ui->tableWidgetInput->item(row, 1)->text().toStdString();
    topic_type.insert(std::make_pair(namet, typet));
  }

  for (int row = 0; row < ui->tableWidgetOutput->rowCount(); ++row)
  {
    std::string name = ui->tableWidgetOutput->item(row, 0)->text().toStdString();
    QLineEdit *line_edit = qobject_cast<QLineEdit *>(ui->tableWidgetOutput->cellWidget(row, 1));
    std::string renamed = line_edit->text().toStdString();
    input_topics.push_back(name);
    std::string selecttype = topic_type.find(name)->second;
    if (selecttype == "sensor_msgs/Image" || selecttype == "sensor_msgs/CompressedImage")
    {
      imgtopic_cnt++;
    }
    if (renamed.empty())
    {
      topis_renamed.insert(std::make_pair(name, name));
    }
    else
    {
      topis_renamed.insert(std::make_pair(name, renamed));
    }
  }

  // double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->dateTime().toMSecsSinceEpoch()));
  // double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));
  double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeEditCurrent->value()));
  double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));

  rosbag::View bag_view(_bag, rosbag::TopicQuery(input_topics),
                        ros::Time(begin_time), ros::Time(end_time));
  rosbag::View bag_view_t(_bag, rosbag::TopicQuery(input_topics));

  play_count_ = bag_view_t.size();
  ui->horizontalSlider->setMaximum(play_count_ - 1);

  bool is_flip, is_mirror, is_fisheye;
  is_flip = ui->checkBoxFlip->isChecked();
  is_mirror = ui->checkBoxMirror->isChecked();
  is_fisheye = ui->checkBoxFisheye->isChecked();

  boost::shared_ptr<velodyne_rawdata::RawData> data_ = boost::make_shared<velodyne_rawdata::RawData>();
  QString calibfile = ui->lineEditVeloCalib->text();
  if (calibfile.isEmpty())
  {
    int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file not found, convert packets need calib, continue anyway?",
                                    QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
    if (ret == QMessageBox::Close)
    {
      return;
    }
    // calibfile="/home/nn1/catkin2/HDL-32.yaml";
  }
  else
  {
    int errret = data_->setupOffline(calibfile.toStdString(), 130, 0.4);
    if (errret < 0)
    {
      int ret = QMessageBox::question(this, "Notice", "Velodyne calibration file load error, continue anyway?",
                                      QMessageBox::Close | QMessageBox::Ok, QMessageBox::Close);
      if (ret == QMessageBox::Close)
      {
        return;
      }
    }
    data_->setParameters(0.4, 130.0, 0, 2 * 3.141592653);
  }

  bool sync_lock = false;
  ros::Time::init();
  ros::Time imgtime1_;
  ros::Time imgtime2_;
  ros::Time pctime1_;
  ros::Time pctime2_;

  // int skipnum = int(ui->spinBoxGrid->value() / 100);
  // int velocnt = 0;

  std::map<std::string, std::vector<std::pair<ros::Time, cv::Mat>>> msgcache;                    //多个相机图片
  std::vector<std::pair<ros::Time, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> pccache; //单个点云
  ros::Time pctime;
  std::vector<ros::Time> imgtimevc;
  for (const rosbag::MessageInstance &msg : bag_view)
  {

    ui->horizontalSlider->setValue(ui->horizontalSlider->value() + 1); //时间轴修改

    // processSlider();
    const auto &name = topis_renamed.find(msg.getTopic())->second;
    // std::cout << msg.getTopic() << " " << msg.getDataType() << std::endl;

    // std::map<std::string,std::vector<std::pair<std::string,std::string>>> msgcache;
    if (msg.getDataType() == "sensor_msgs/Image" || msg.getDataType() == "sensor_msgs/CompressedImage")
    {
      cv::Mat current_image;
      if (msg.getDataType() == "sensor_msgs/Image")
      {
        sensor_msgs::Image::Ptr in_image_msg = msg.instantiate<sensor_msgs::Image>();
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
        current_image = cv_image->image;
        imgtime1_ = in_image_msg->header.stamp;
      }
      else if (msg.getDataType() == "sensor_msgs/CompressedImage")
      {

        sensor_msgs::CompressedImage::Ptr in_image_msg = msg.instantiate<sensor_msgs::CompressedImage>();
        QString imgtype = QString::fromStdString(name).split("/").at(2);
        if (imgtype == "image_h264")
        {

          AVPacket packet;
          av_init_packet(&packet);
          packet.data = const_cast<uint8_t *>(in_image_msg->data.data());
          packet.size = in_image_msg->data.size();
          packet.pts = AV_NOPTS_VALUE;
          packet.dts = AV_NOPTS_VALUE;

          AVFrame frame;
          memset(&frame, 0, sizeof(frame));

          int gotPicture;

          if (avcodec_decode_video2(g_codec, &frame, &gotPicture, &packet) < 0)
          {
            continue;
          }

          if (gotPicture)
          {

            g_sws = sws_getCachedContext(
                g_sws,
                frame.width, frame.height, AV_PIX_FMT_YUV420P,
                frame.width, frame.height, AV_PIX_FMT_RGB24,
                0, 0, 0, 0);

            sensor_msgs::ImagePtr img(new sensor_msgs::Image);

            img->encoding = "rgb8";
            img->data.resize(frame.width * frame.height * 3);
            img->step = frame.width * 3;
            img->width = frame.width;
            img->height = frame.height;
            img->header.frame_id = "cam";
            img->header.stamp = ros::Time::now(); // FIXME

            uint8_t *destData[1] = {img->data.data()};
            int linesize[1] = {(int)img->step};

            sws_scale(g_sws, frame.data, frame.linesize, 0, frame.height,
                      destData, linesize);

            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            current_image = cv_image->image;
          }
        }
        else
        {
          current_image = cv::imdecode(cv::Mat(in_image_msg->data), 1);
        }
        imgtime1_ = in_image_msg->header.stamp;
      }

      //将每张图添加到map
      auto iter = msgcache.find(name);
      if (iter == msgcache.end())
      {
        std::vector<std::pair<ros::Time, cv::Mat>> vectertmp;
        vectertmp.push_back(std::make_pair(imgtime1_, current_image));
        // std::cout<<"f"<<vectertmp.size()<<std::endl;
        msgcache.insert(std::make_pair(name, vectertmp));
      }
      else
      {
        std::vector<std::pair<ros::Time, cv::Mat>> vectertmp;
        vectertmp.swap(iter->second);
        vectertmp.push_back(std::make_pair(imgtime1_, current_image));
        // std::cout<<"s"<<vectertmp.size()<<" "<<in_image_msg->header.stamp<<std::endl;
        msgcache[name].swap(vectertmp);
      }

      std::vector<ros::Time>::iterator imgtime;
      std::vector<std::pair<ros::Time, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> pcvaluetmp;

      if (imgtimevc.size() > 0)
      {
        imgtime = std::max_element(imgtimevc.begin(), imgtimevc.end());
        //将时间小于图像时间的点云删除
        if (sync_lock == false) //没锁就删
        {
          // std::cout << "pc:" << pccache.size() << " " << pccache[0].first << " img2 " << msgcache2.size() << std::endl;
          for (auto itemp = pccache.begin(); itemp != pccache.end(); itemp++)
          {
            std::cout << "pc keep:" << itemp->first << " " << (itemp->first - *imgtime).toNSec() << " " << (itemp->first - *imgtime).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() << std::endl;

            if ((itemp->first - *imgtime).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() > 0)
            {
              // value.erase(itemv);
              pcvaluetmp.push_back(*itemp);
            }
            // else
            // {

            // }
            pctime = pccache.begin()->first;
          }
          if (pcvaluetmp.size() > 0)
          {
            pctime = pcvaluetmp.begin()->first;
            pccache.assign(pcvaluetmp.begin(), pcvaluetmp.end());
            sync_lock = true; //删了就加锁
          }
        }
      }

      // std::cout << "pccache.size() " << pccache.size() << " msgcache.size() " << msgcache.size() << std::endl;
      std::map<std::string, std::vector<std::pair<ros::Time, cv::Mat>>> msgcache2;
      if (pccache.size() > 0)
      {
        imgtimevc.empty();
        for (auto item = msgcache.begin(); item != msgcache.end(); item++)
        {
          std::vector<std::pair<ros::Time, cv::Mat>> valuetmp;
          auto key = item->first;
          auto value = item->second;
          // auto lastitemv=value.begin();
          for (auto itemv = value.begin(); itemv != value.end(); itemv++)
          {
            //对于图片时间小于点云删除
            // if(abs((pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt())<70){
            //   std::cout << "img keep:" << pctime << " " << (pctime - itemv->first).toNSec() << " " << (pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() << std::endl;
            // }
            // std::cout<<(pctime - lastitemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() <<" "<<(pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() <<std::endl;
            if ((pctime - itemv->first).toNSec() / 1000000 - ui->spinBoxDelay->text().toInt() < 0)
            {
              // value.erase(itemv);
              valuetmp.push_back(*itemv);
            }
            // lastitemv=itemv;
          }
          //每个字典的元素长度
          // std::cout <<"-key: "<<key<< " size: " << value.size() << std::endl;

          if (valuetmp.size() > 0)
          {
            // std::cout << "img:" << key << " " << valuetmp.size() << " " << valuetmp[0].first << std::endl;
            // sync_lock = true;
            //添加到新的字典
            msgcache2.insert(std::make_pair(key, valuetmp));
            imgtimevc.push_back(valuetmp[0].first);
          }
        }
      }
      //交换图像字典数据
      msgcache.swap(msgcache2);

      //没锁就不写，没达到预定数量图片不写
      std::cout << "msgcache size: " << msgcache.size() << " imgtopic_cnt:" << imgtopic_cnt << std::endl;
      if (sync_lock == false || msgcache.size() != imgtopic_cnt)
      {
        continue;
      }
      //显示或写入磁盘

      for (auto item = msgcache.begin(); item != msgcache.end(); item++)
      {
        // std::cout << "-img cnt:" << item->second.size() << " pc cnt:" << pccache.size() << " sync:" << sync_lock << std::endl;
        if (item->second.size() == 0 || pccache.size() == 0)
        {
          continue;
        }

        cv::Mat current_image_t;
        item->second[0].second.copyTo(current_image_t);
        imgtime1_ = ros::Time::now();
        imgtime2_ = item->second[0].first;
        std::cout << "-pc time:" << pctime << " img time:" << imgtime2_ << " delay:" << pctime - imgtime2_ << std::endl;
        //对比点云图像时间

        //翻转镜像
        if (is_flip)
        {
          cv::flip(current_image_t, current_image_t, 0);
        }
        if (is_mirror)
        {
          cv::flip(current_image_t, current_image_t, 1);
        }

        //畸变校正
        std::shared_ptr<cv::Mat> img_marked = std::make_shared<cv::Mat>();
        if (flag_kd_ == 0)
        {
          current_image_t.copyTo(*img_marked);
        }
        else
        {
          if (is_fisheye)
          {
            cv::Mat D;
            D_.copyTo(D);
            D.pop_back();
            //        std::cout<<K_<<D<<std::endl;
            cv::Matx33d newK = K_;
            cv::fisheye::undistortImage(current_image_t, *img_marked, K_, D, newK);
          }
          else
          {
            cv::undistort(current_image_t, *img_marked, K_, D_);
          }
        }
        // continue;

        //投影
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcc(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*pccache.begin()->second, *pcc);
        for (auto &p : pcc->points)
        {
          p.rgba = 0xffffffff;
        }
        std::shared_ptr<cv::Mat> img_mark = std::make_shared<cv::Mat>();
        img_marked->copyTo(*img_mark);

        calibrator_->Project(*pcc, *img_mark);

        // img_viewer_->showImage(img_mark);

        //写入
        std::stringstream ss1;
        ss1 << outputdir.toStdString() << "/" << QString::fromStdString(item->first).split("/").at(1).toStdString() << "/";
        QString fullPath1 = QString::fromStdString(ss1.str());
        QDir imgdir1(fullPath1);
        if (!imgdir1.exists())
        {
          bool ok = imgdir1.mkpath(fullPath1); //创建多级目录
        }
        ss1 << imgtime2_.toNSec() / 1000000 << "_";
        ss1 << ((int)imgtime2_.toNSec() - (int)pccache.begin()->first.toNSec()) / 1000000 << ".jpg";
        std::string file_path1 = ss1.str();
        cv::imwrite(file_path1, current_image_t);
        std::cout << file_path1 << std::endl;

        std::stringstream ss2;
        ss2 << outputdir.toStdString() << "/pointcloud/";
        QString fullPath2 = QString::fromStdString(ss2.str());
        QDir imgdir2(fullPath2);
        if (!imgdir2.exists())
        {
          bool ok = imgdir2.mkpath(fullPath2); //创建多级目录
        }
        ss2 << imgtime2_.toNSec() / 1000000 << "_";
        ss2 << ((int)imgtime2_.toNSec() - (int)pccache.begin()->first.toNSec()) / 1000000 << ".pcd";
        std::string file_path2 = ss2.str();
        std::cout << file_path2 << std::endl;
        if (!QFile::exists(QString::fromStdString(file_path2)))
        {
          // save PCD file as well
          pcl::copyPointCloud(*pccache.begin()->second, *pc);
          pcl::io::savePCDFileASCII(file_path2, *pc);
        }

        std::stringstream ss3;
        ss3 << outputdir.toStdString() << "/" << QString::fromStdString(item->first).split("/").at(1).toStdString() + "_merge"
            << "/";
        QString fullPath3 = QString::fromStdString(ss3.str());
        QDir imgdir3(fullPath3);
        if (!imgdir3.exists())
        {
          bool ok = imgdir3.mkpath(fullPath3); //创建多级目录
        }
        ss3 << imgtime2_.toNSec() / 1000000 << "_";
        ss3 << ((int)imgtime2_.toNSec() - (int)pccache.begin()->first.toNSec()) / 1000000 << ".jpg";
        std::string file_path3 = ss3.str();
        cv::imwrite(file_path3, *img_mark);
        std::cout << file_path3 << std::endl;

        sync_lock = false;
        cv::waitKey(1);
      }
    }
    else if (msg.getDataType() == "velodyne_msgs/VelodyneScan")
    {
      // velocnt++;
      // if (velocnt == skipnum)
      // {
      //   velocnt = 0;
      // }
      // else
      // {
      //   continue;
      // }
      velodyne_msgs::VelodyneScan::Ptr in_sensor_cloud = msg.instantiate<velodyne_msgs::VelodyneScan>();
      // msgcache.insert(std::make_pair(name,std::make_pair(in_sensor_cloud->header.stamp,*msg)));

      velodyne_rawdata::VPointCloud::Ptr
          outMsg(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(in_sensor_cloud->header).stamp;
      outMsg->header.frame_id = in_sensor_cloud->header.frame_id;
      outMsg->height = 1;

      pctime1_ = ros::Time::now();
      pctime2_ = in_sensor_cloud->header.stamp;

      std::cout << in_sensor_cloud->header.stamp << " " << msg.getTime() << std::endl;

      // process each packet provided by the driver
      for (size_t i = 0; i < in_sensor_cloud->packets.size(); ++i)
      {
        data_->unpack(in_sensor_cloud->packets[i], *outMsg);
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr pctmp(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud(*outMsg, *pctmp);
      pccache.push_back(std::make_pair(in_sensor_cloud->header.stamp, pctmp));
    }
    else if (msg.getDataType() == "sensor_msgs/PointCloud2")
    {
      // velocnt++;
      // if (velocnt == skipnum)
      // {
      //   velocnt = 0;
      // }
      // else
      // {
      //   continue;
      // }
      sensor_msgs::PointCloud2::Ptr in_sensor_cloud = msg.instantiate<sensor_msgs::PointCloud2>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*in_sensor_cloud, *velodyne_cloud_ptr);

      //save PCD file as well
      // pcl::io::savePCDFileASCII(file_path, *velodyne_cloud_ptr);
      pccache.push_back(std::make_pair(in_sensor_cloud->header.stamp, velodyne_cloud_ptr));
    }
    if (paused_)
    {
      break;
    }
    // break;
  }
  int ret = QMessageBox::question(this, "Done", "Convert succesfully created. Do you want to close the application?",
                                  QMessageBox::Cancel | QMessageBox::Close, QMessageBox::Close);
  if (ret == QMessageBox::Close)
  {
    this->close();
  }
}

void RosbagEditor::on_pushButtonVeloCalib_pressed()
{
  QString filename = QFileDialog::getOpenFileName(this,
                                                  tr("Open Velodyne Calibration File"), _previous_load_path, tr("Calib Files (*.yaml)"));

  if (filename.isEmpty())
  {
    return;
  }

  ui->lineEditVeloCalib->setText(filename);
}

void RosbagEditor::on_pushButtonSave_pressed()
{
  QString filename = QFileDialog::getSaveFileName(this, "Save Rosbag",
                                                  _previous_save_path,
                                                  tr("Rosbag Files (*.bag)"));
  if (filename.isEmpty())
  {
    return;
  }
  if (QFileInfo(filename).suffix() != "bag")
  {
    filename.append(".bag");
  }

  if (filename == _loade_filename)
  {
    QMessageBox::warning(this, "Wrong file name",
                         "You can not overwrite the input file. Choose another name or directory.",
                         QMessageBox::Ok);
    return;
  }

  QSettings settings;
  settings.setValue("RosbagEditor/prevSavePath", QFileInfo(filename).absolutePath());

  rosbag::Bag out_bag;

  out_bag.open(filename.toStdString(), rosbag::bagmode::Write);

  if (ui->radioNoCompression->isChecked())
  {
    out_bag.setCompression(rosbag::CompressionType::Uncompressed);
  }
  else if (ui->radioLZ4->isChecked())
  {
    out_bag.setCompression(rosbag::CompressionType::LZ4);
  }
  else if (ui->radioBZ2->isChecked())
  {
    out_bag.setCompression(rosbag::CompressionType::BZ2);
  }

  std::vector<std::string> input_topics;
  std::map<std::string, std::string> topis_renamed;

  for (int row = 0; row < ui->tableWidgetOutput->rowCount(); ++row)
  {
    std::string name = ui->tableWidgetOutput->item(row, 0)->text().toStdString();
    QLineEdit *line_edit = qobject_cast<QLineEdit *>(ui->tableWidgetOutput->cellWidget(row, 1));
    std::string renamed = line_edit->text().toStdString();
    input_topics.push_back(name);
    if (renamed.empty())
    {
      topis_renamed.insert(std::make_pair(name, name));
    }
    else
    {
      topis_renamed.insert(std::make_pair(name, renamed));
    }
  }

  double begin_time = std::floor(-0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputBegin->dateTime().toMSecsSinceEpoch()));
  double end_time = std::ceil(0.001 + 0.001 * static_cast<double>(ui->dateTimeOutputEnd->dateTime().toMSecsSinceEpoch()));

  rosbag::View bag_view(_bag, rosbag::TopicQuery(input_topics),
                        ros::Time(begin_time), ros::Time(end_time));

  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality(Qt::ApplicationModal);
  progress_dialog.show();

  int msg_count = 0;
  progress_dialog.setRange(0, bag_view.size() - 1);

  bool do_tf_filtering = _filtered_frames.size() > 0 && ui->checkBoxFilterTF->isChecked();

  for (const rosbag::MessageInstance &msg : bag_view)
  {
    if (msg_count++ % 100 == 0)
    {
      progress_dialog.setValue(msg_count);
      QApplication::processEvents();
    }

    const auto &name = topis_renamed.find(msg.getTopic())->second;

    auto removeTransform = [&](std::vector<geometry_msgs::TransformStamped> &transforms) {
      for (int i = 0; i < transforms.size(); i++)
      {
        auto frame = std::make_pair(transforms[i].header.frame_id,
                                    transforms[i].child_frame_id);
        if (_filtered_frames.count(frame))
        {
          transforms.erase(transforms.begin() + i);
          i--;
        }
      }
    };

    if (msg.getTopic() == "/tf" && do_tf_filtering)
    {
      tf::tfMessage::Ptr tf = msg.instantiate<tf::tfMessage>();
      if (tf)
      {
        removeTransform(tf->transforms);
        out_bag.write(name, msg.getTime(), tf, msg.getConnectionHeader());
      }

      tf2_msgs::TFMessage::Ptr tf2 = msg.instantiate<tf2_msgs::TFMessage>();
      if (tf2)
      {
        removeTransform(tf2->transforms);
        out_bag.write(name, msg.getTime(), tf2, msg.getConnectionHeader());
      }
    }
    else
    {
      out_bag.write(name, msg.getTime(), msg, msg.getConnectionHeader());
    }
  }
  out_bag.close();

  int ret = QMessageBox::question(this, "Done", "New robag succesfully created. Do you want to close the application?",
                                  QMessageBox::Cancel | QMessageBox::Close, QMessageBox::Close);
  if (ret == QMessageBox::Close)
  {
    this->close();
  }
}

void RosbagEditor::on_dateTimeOutputEnd_dateTimeChanged(const QDateTime &dateTime)
{
  if (ui->dateTimeOutputBegin->dateTime() > dateTime)
  {
    ui->dateTimeOutputBegin->setDateTime(dateTime);
  }
}

void RosbagEditor::on_dateTimeOutputBegin_dateTimeChanged(const QDateTime &dateTime)
{
  if (ui->dateTimeOutputEnd->dateTime() < dateTime)
  {
    ui->dateTimeOutputEnd->setDateTime(dateTime);
  }
}

void RosbagEditor::on_checkBoxFilterTF_toggled(bool checked)
{
  bool contains_tf = !ui->tableWidgetInput->findItems("/tf", Qt::MatchExactly).empty();
  ui->pushButtonFilterTF->setEnabled(checked && contains_tf);
}

void RosbagEditor::on_pushButtonFilterTF_pressed()
{
  FilterFrames dialog(_bag, _filtered_frames, this);
  dialog.exec();
}
