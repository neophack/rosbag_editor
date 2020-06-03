#ifndef ROSBAG_EDITOR_H
#define ROSBAG_EDITOR_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QDir>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <QSettings>

#include "imageviewer.h"

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include "calibrator.h"
#include "json.hpp"
#include "pointcloudviewer.h"


extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace Ui {
class RosbagEditor;
}

class RosbagEditor : public QMainWindow
{
    Q_OBJECT

public:
    explicit RosbagEditor(QWidget *parent = nullptr);
    ~RosbagEditor();

private slots:
    void closeEvent(QCloseEvent *event);

    void on_pushButtonLoad_pressed();

    void on_pushButtonMove_pressed();

    void on_tableWidgetInput_itemSelectionChanged();

    void on_tableWidgetOutput_itemSelectionChanged();

    void on_pushButtonRemove_pressed();

    void on_pushButtonSave_pressed();

    void on_pushButtonVeloCalib_pressed();

    void on_pushButtonConvert_pressed();

    void on_pushButtonPlay_pressed();

    void on_pushButtonNext_pressed();

    void on_pushButtonPause_pressed();

    void on_pushButtonSnapshot_pressed();

    void on_dateTimeOutputEnd_dateTimeChanged(const QDateTime &dateTime);

    void on_dateTimeOutputBegin_dateTimeChanged(const QDateTime &dateTime);

    void on_checkBoxFilterTF_toggled(bool checked);

    void on_pushButtonFilterTF_pressed();

    void PerviewSlot(int c,int r);

    void processSlider();

    void on_pushButtonUpdateSlider_pressed();

    void on_pushButtonLidarCamCalib_pressed();

    void on_pushButtonConvertAll_pressed();

    void on_pushButtonSnapshotAll_pressed();


   private:
    Ui::RosbagEditor *ui;
    QString _loade_filename;
    QString _previous_load_path;
    QString _previous_save_path;
    bool sync_topics_;
    size_t image_frame_counter_, cloud_frame_counter_;
    rosbag::Bag _bag;
    std::set<std::pair<std::string,std::string>> _filtered_frames;
    void changeEnabledWidgets();
    std::unique_ptr<ImageViewer> img_viewer_;
    std::unique_ptr<PointcloudViewer> pc_viewer_;
    std::unique_ptr<lqh::Calibrator> calibrator_;
    bool paused_;
    double_t sec_begin_;
    double_t sec_end_;
    int32_t play_count_;
    nlohmann::json js_;

    uint8_t flag_kd_=0;
    cv::Mat K_;
    cv::Mat D_;
    bool snapshot_=false;

    AVCodecContext* g_codec = 0;
    SwsContext* g_sws = 0;
};

#endif // ROSBAG_EDITOR_H
