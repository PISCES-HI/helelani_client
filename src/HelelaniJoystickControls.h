#ifndef HELELANIJOYSTICKCONTROLS_HPP
#define HELELANIJOYSTICKCONTROLS_HPP
#include <pluginlib/class_list_macros.h>
#include <helelani_common/Throttle.h>
#include <helelani_common/CameraCtrl.h>
#include "ui_HelelaniJoystickControls.h"

#include <QMainWindow>
#include <QtCore>
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

struct js_event {
        unsigned int time;      /* event timestamp in milliseconds */
        short value;   /* value */
        unsigned char type;     /* event type */
        unsigned char number;   /* axis/button number */
};

namespace helelani_client {

class HelelaniJoystickControls : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
    QThread workerThread;

public:
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
public slots:
    void watcher();
    void toggle();
    void pushVals(unsigned char type, unsigned char number, short val);
    void checkWindowSize();
    void checkSitCam();
signals:
    void gamepadupdate();
    void operate();
private:
    void changeThrottles(float left, float right);
    void changeCam(float p, float t);
    void sitcamCallback(const helelani_common::CameraCtrl& msg);
    Ui::HelelaniJoystickControls m_ui;
    QWidget* m_widget = nullptr;
    ros::Publisher m_pub;
    ros::Publisher m_situationCamPub;
    ros::Subscriber m_situationCamSub;

    float pan = 90;
    float tilt = 90;

    int pan_inc = 0;
    int tilt_inc = 0;
    
    //base button (x,y) location values
    float LTB_Button_x = 166.f; float LTB_Button_y = 264.f;
    float RTB_Button_x = 419.f; float RTB_Button_y = 264.f;
    float dir_pad_x = 34.f; float dir_pad_y = 152.f;
    float a_button_x = 546.f; float a_button_y = 213.f;
    float b_button_x = 582.f; float b_button_y = 177.f;
    float x_button_x = 510.f; float x_button_y = 177.f;
    float y_button_x = 546.f; float y_button_y = 141.f;
    float start_button_x = 404.f; float start_button_y = 141.f;
    float back_button_x = 212.f; float back_button_y = 141.f;
    float LTB_Placeholder_x = 161.f; float LTB_Placeholder_y = 259.f;
    float RTB_Placeholder_x = 414.f; float RTB_Placeholder_y = 259.f;
    float LB_x = 9.f; float LB_y = 65.f;
    float LT_x = 9.f; float LT_y = 9.f;
    float RB_x = 581.f; float RB_y = 65.f;
    float RT_x = 581.f; float RT_y = 9.f;
    float logo_Button_x = 285.f; float logo_Button_y = 150.f;
    float toggleGamepad_x = 253.f; float toggleGamepad_y = 343.f;

    //base GUI values
    float abxy_button_size = 50.f;
    float trigger_size = 75.f;
    float bunker_size = 75.f;
    float start_back_button_size = 50.f;
    float logo_button_size = 100.f;
    float Dpad_size = 120.f;

    float toggleGamepadSize_x = 160.f;
    float toggleGamepadSize_y = 25.f;

    float thumb_bottom_layer_size = 90.f;
    float thumb_top_layer_size = 80.f;
    float size_offset = 5.f;//based on difference of thumb bottom and top layer (visual) sizes (Ex: ads(90-80)=10, so 10/2 = 5 = size_offset)

    float default_window_size_x = 667.f;
    float default_window_size_y = 398.f;
    float def_window_hyp = sqrt((default_window_size_x*default_window_size_x)+(default_window_size_y*default_window_size_y));//diagonal

    //changing GUI values
    float new_window_size_x = default_window_size_x;
    float new_window_size_y = default_window_size_y;
    float old_window_size_x = new_window_size_x;
    float old_window_size_y = new_window_size_y;

    float current_window_hyp = def_window_hyp;
    float mult_factor = 1.f;
    float mult_factor_x = 1.f;
    float mult_factor_y = 1.f;

    float cur_abxy_size = abxy_button_size;
    float cur_trigger_size = trigger_size;
    float cur_bunker_size = bunker_size;
    float cur_start_back_size = start_back_button_size;
    float cur_logo_size = logo_button_size;
    float cur_Dpad_size = Dpad_size;
    float cur_thumb_bottom_size = thumb_bottom_layer_size;
    float cur_thumb_top_size = thumb_top_layer_size;
    float cur_toggleGamepadSize_x = toggleGamepadSize_x;
    float cur_toggleGamepadSize_y = toggleGamepadSize_y;
};

class Worker : public QObject{
    Q_OBJECT
public slots:
    void doWork();
signals:
    void resultReady(unsigned char type, unsigned char number, short val);
};

}

#endif // HELELANIJOYSTICKCONTROLS_HPP
