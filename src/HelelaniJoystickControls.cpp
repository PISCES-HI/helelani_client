#include "HelelaniJoystickControls.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>

using namespace std;

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

bool usingGamepadControls = true;//change for control between throttlepad and gamepad

float MAX_JOYSTICK_VALUE = 32767.f;
float valVertLeft = 0.f;
float valVertRight = 0.f;
float valHorizLeft = 0.f;
float valHorizRight = 0.f;
int fd;

//Find folder containing image/sprite files for button visuals and map out each button accordingly using QPixmap's. (p = pressed; unp = unpressed; dis = disabled/grey)
QPixmap a_p(":/buttons/360_A_pressed.png");
QPixmap a_unp(":/buttons/360_A_unpressed.png");
QPixmap a_dis(":/buttons/360_A_disabled.png");

QPixmap b_p(":/buttons/360_B_pressed.png");
QPixmap b_unp(":/buttons/360_B_unpressed.png");
QPixmap b_dis(":/buttons/360_B_disabled.png");

QPixmap x_p(":/buttons/360_X_pressed.png");
QPixmap x_unp(":/buttons/360_X_unpressed.png");
QPixmap x_dis(":/buttons/360_X_disabled.png");

QPixmap y_p(":/buttons/360_Y_pressed.png");
QPixmap y_unp(":/buttons/360_Y_unpressed.png");
QPixmap y_dis(":/buttons/360_Y_disabled.png");

QPixmap start_p(":/buttons/360_Start_pressed.png");
QPixmap start_unp(":/buttons/360_Start_unpressed.png");
QPixmap start_dis(":/buttons/360_Start_disabled.png");

QPixmap back_p(":/buttons/360_Back_pressed.png");
QPixmap back_unp(":/buttons/360_Back_unpressed.png");
QPixmap back_dis(":/buttons/360_Back_disabled.png");

QPixmap LB_p(":/buttons/360_LB_pressed.png");
QPixmap LB_unp(":/buttons/360_LB_unpressed.png");
QPixmap LB_dis(":/buttons/360_LB_disabled.png");

QPixmap LT_p(":/buttons/360_LT_pressed.png");
QPixmap LT_unp(":/buttons/360_LT_unpressed.png");
QPixmap LT_dis(":/buttons/360_LT_disabled.png");

QPixmap RB_p(":/buttons/360_RB_pressed.png");
QPixmap RB_unp(":/buttons/360_RB_unpressed.png");
QPixmap RB_dis(":/buttons/360_RB_disabled.png");

QPixmap RT_p(":/buttons/360_RT_pressed.png");
QPixmap RT_unp(":/buttons/360_RT_unpressed.png");
QPixmap RT_dis(":/buttons/360_RT_disabled.png");

QPixmap LTB_p(":/buttons/360_Left_Stick_Top_layer_pressed.png");
QPixmap LTB_unp(":/buttons/360_Left_Stick_Top_layer_unpressed.png");
QPixmap LTB_dis(":/buttons/360_Left_Stick_Top_layer_disabled.png");

QPixmap RTB_p(":/buttons/360_Right_Stick_Top_layer_pressed.png");
QPixmap RTB_unp(":/buttons/360_Right_Stick_Top_layer_unpressed.png");
QPixmap RTB_dis(":/buttons/360_Right_Stick_Top_layer_disabled.png");

QPixmap LTB_plc(":/buttons/360_Left_Stick_Bottom_layer.png");
QPixmap RTB_plc(":/buttons/360_Right_Stick_Bottom_layer.png");

QPixmap dir_plc(":/buttons/360_Dpad.png");
QPixmap dir_dis(":/buttons/360_Dpad_disabled.png");
QPixmap dir_dwn_unpressed(":/buttons/360_Dpad_Down_unpressed.png");
QPixmap dir_dwn_pressed(":/buttons/360_Dpad_Down_pressed.png");
QPixmap dir_dwn_dis(":/buttons/360_Dpad_Down_disabled.png");
QPixmap dir_up_unpressed(":/buttons/360_Dpad_Up_unpressed.png");
QPixmap dir_up_pressed(":/buttons/360_Dpad_Up_pressed.png");
QPixmap dir_up_dis(":/buttons/360_Dpad_Up_disabled.png");
QPixmap dir_left_unpressed(":/buttons/360_Dpad_Left_unpressed.png");
QPixmap dir_left_pressed(":/buttons/360_Dpad_Left_pressed.png");
QPixmap dir_left_dis(":/buttons/360_Dpad_Left_disabled.png");
QPixmap dir_right_unpressed(":/buttons/360_Dpad_Right_unpressed.png");
QPixmap dir_right_pressed(":/buttons/360_Dpad_Right_pressed.png");
QPixmap dir_right_dis(":/buttons/360_Dpad_Right_disabled.png");

QPixmap logo_p(":/buttons/logo_pressed.png");
QPixmap logo_unp(":/buttons/logo_unpressed.png");
QPixmap logo_dis(":/buttons/logo_disabled.png");

namespace helelani_client {

void HelelaniJoystickControls::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();

    // create QWidget
    m_widget = new QWidget();

    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    m_widget->setObjectName("HelelaniJoystickControls");
    if (context.serialNumber() > 1)
        m_widget->setWindowTitle(m_widget->windowTitle() +
                                 " (" + QString::number(context.serialNumber()) + ")");
    
    //initial setup for graphical QPixmap buttons
    m_ui.a_Button->setPixmap(a_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
    m_ui.b_Button->setPixmap(b_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
    m_ui.x_Button->setPixmap(x_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
    m_ui.y_Button->setPixmap(y_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
    m_ui.start_Button->setPixmap(start_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
    m_ui.back_Button->setPixmap(back_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
    m_ui.LB->setPixmap(LB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
    m_ui.LT->setPixmap(LT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
    m_ui.RB->setPixmap(RB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
    m_ui.RT->setPixmap(RT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));

    m_ui.dir_pad->setPixmap(dir_plc.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));

    m_ui.dir_down->setPixmap(dir_dwn_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
    m_ui.dir_up->setPixmap(dir_up_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
    m_ui.dir_left->setPixmap(dir_left_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
    m_ui.dir_right->setPixmap(dir_right_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));

    m_ui.LTB_Button->setPixmap(LTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
    m_ui.RTB_Button->setPixmap(RTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));

    m_ui.LTB_Placeholder->setPixmap(LTB_plc.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));
    m_ui.RTB_Placeholder->setPixmap(RTB_plc.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));

    m_ui.logo_Button->setPixmap(logo_unp.scaled(cur_logo_size,cur_logo_size,Qt::KeepAspectRatio));

    //Qimage testing
    QImage qimagetest(":/buttons/360_A_unpressed.png");
    QPixmap dummy/*(qimagetest.size())*/;
    
    /* METHOD 1: PAINTER OBJECT
    QPainter p;
    p.begin(&dummy);
    p.setCompositionMode(QPainter::CompositionMode_Source);
    p.drawPixmap(0, 0, QPixmap::fromImage(qimagetest));
    p.setCompositionMode(QPainter::CompositionMode_DestinationIn);
  illRect(dummy.rect(), QColor(0,0,0,100));
    p.end();
    */
    // METHOD 2: SCAN
    for(int y = 0; y < qimagetest.height(); ++y){
        QRgb *row = (QRgb*)qimagetest.scanLine(y);
        for(int x = 0; x < qimagetest.width(); ++x){
		if( ((unsigned char*)&row[x])[3] != 0)
		((unsigned char*)&row[x])[3] = 105;
        }
    }
    dummy.convertFromImage(qimagetest);
    //m_ui.test_Button->setPixmap(dummy.scaled(cur_logo_size,cur_logo_size,Qt::KeepAspectRatio));

    // add widget (window) to the user interface
    context.addWidget(m_widget);
    
    //Stack Placeholder visuals under the Thumbstick Button visuals (quality control)
    m_ui.LTB_Placeholder->lower();
    m_ui.RTB_Placeholder->lower();
    m_ui.dir_pad->lower();

    // timer architecture
    QTimer *timer = new QTimer(this);
    timer->start(50);
    //std::printf("starting timer\n");
    connect(timer, SIGNAL(timeout()), this, SLOT(checkWindowSize()));
    connect(timer, SIGNAL(timeout()), this, SLOT(checkSitCam()));
    // signal architecture
    connect(m_ui.toggleGamepad, SIGNAL(clicked()), this, SLOT(toggle()));
    //connect(m_widget, SIGNAL(resizeEvent()), this, SLOT(resize()));

    // publishing
    ros::NodeHandle n;
    m_pub = n.advertise<helelani_common::Throttle>("/helelani/throttle", 10);
    m_situationCamPub = n.advertise<helelani_common::CameraCtrl>("/helelani/situation_cam_ctrl", 1000);
    m_situationCamSub = n.subscribe("/helelani/situation_cam_ctrl", 10, &HelelaniJoystickControls::sitcamCallback, this);

    // joystick
    fd = open ("/dev/input/js0", O_RDONLY);//<-----------------------------------finds first joystick ('js0')
	if (fd < 0){
		printf("cannot open device\n");}
	else{
		printf("open success\n");}

    // thread
    Worker *worker = new Worker;
    worker->moveToThread(&workerThread);
    connect(this, &HelelaniJoystickControls::operate, worker, &Worker::doWork);
    connect(worker, &Worker::resultReady, this, &HelelaniJoystickControls::pushVals);
    workerThread.start();
    emit operate();//start working
}

void HelelaniJoystickControls::shutdownPlugin()
{
    m_pub.shutdown();
    m_situationCamPub.shutdown();
}

void HelelaniJoystickControls::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                       qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniJoystickControls::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                          const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniJoystickControls::sitcamCallback(const helelani_common::CameraCtrl& msg){
	pan = msg.pan;
	tilt = msg.tilt;
	//printf("pan: %f, tilt: %f\n", pan, tilt);
}

void HelelaniJoystickControls::pushVals(unsigned char type, unsigned char number, short val){
		
	if (type == JS_EVENT_AXIS && usingGamepadControls){ 	//checking for only axis values, not buttons, while using controls.
		if((int) number == 0){		//L thumbstick tilt (x-direction, normal value)
		    	if(val >= -5000 && val <= 5000){
		    		valHorizLeft = 0.f;
		    	}
		    	else{
		    		valHorizLeft = (val/MAX_JOYSTICK_VALUE) * -1.f;
		    	}
			m_ui.LTB_Button->move((m_ui.LTB_Placeholder->x() + size_offset * mult_factor) - (valHorizLeft * 25.f * mult_factor), (m_ui.LTB_Placeholder->y() + size_offset * mult_factor) - (valVertLeft * 25.f * mult_factor));
			//Print the (x,y) position of this GUI button based on the window.
			//std::printf("Left Thumb Stick: %d x, %d y\n", m_ui.LTB_Button->x(), m_ui.LTB_Button->y());
		}
		if((int) number == 1){		//L thumbstick tilt (y-direction, inverted value)
		    	if(val >= -5000 && val <= 5000){
		    		valVertLeft = 0.f;
		    	}
		    	else{
		    		valVertLeft = (val/MAX_JOYSTICK_VALUE) * -1.f;
		    	}
			m_ui.LTB_Button->move((m_ui.LTB_Placeholder->x() + size_offset * mult_factor) - (valHorizLeft * 25.f * mult_factor), (m_ui.LTB_Placeholder->y() + size_offset * mult_factor) - (valVertLeft * 25.f * mult_factor));
			//std::printf("Left Thumb Stick: %d x, %d y\n", m_ui.LTB_Button->x(), m_ui.LTB_Button->y());
		}
		if((int) number == 2){		//L trigger
		    	if(val <= -30000){
		    		m_ui.LT->setPixmap(LT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.LT->setPixmap(LT_p.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("LT: %d x, %d y\n", m_ui.LT->x(), m_ui.LT->y());
		}
		if((int) number == 3){		//R thumbstick tilt (x-direction, normal value)
		    	if(val >= -5000 && val <= 5000){
		    		valHorizRight = 0.f;
		    	}
		    	else{
		    		valHorizRight = (val/MAX_JOYSTICK_VALUE) * -1.f;
		    	}
			m_ui.RTB_Button->move((m_ui.RTB_Placeholder->x() + size_offset * mult_factor) - (valHorizRight * 25.f * mult_factor), (m_ui.RTB_Placeholder->y() + size_offset * mult_factor) - (valVertRight * 25.f * mult_factor));
			//std::printf("Right Thumb Stick: %d x, %d y\n", m_ui.LTB_Button->x(), m_ui.LTB_Button->y());
		}
		if ((int) number == 4){		//R thumbstick tilt (y-direction, inverted value)
		    	if(val >= -5000 && val <= 5000){
		    		valVertRight = 0.f;
		    	}
		    	else{
		    		valVertRight = (val/MAX_JOYSTICK_VALUE) * -1.f;
			}
			m_ui.RTB_Button->move((m_ui.RTB_Placeholder->x() + size_offset * mult_factor) - (valHorizRight * 25.f * mult_factor), (m_ui.RTB_Placeholder->y() + size_offset * mult_factor) - (valVertRight * 25.f * mult_factor));
			//std::printf("Right Thumb Stick: %d x, %d y\n", m_ui.RTB_Button->x(), m_ui.RTB_Button->y());
		}
		if((int) number == 5){		//R trigger
		    	if(val <= -30000){
		    		m_ui.RT->setPixmap(RT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.RT->setPixmap(RT_p.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("RT: %d x, %d y\n", m_ui.RT->x(), m_ui.RT->y());
		}
		if((int) number == 6){		//LEFT or RIGHT - directional pad
		    	if(val >= 30000){
    				m_ui.dir_right->setPixmap(dir_right_pressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
				pan_inc = 1;
		    	}
		    	else if (val == 0){
				pan_inc = 0;
		    		m_ui.dir_right->setPixmap(dir_right_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		    	}
			if(val <= -30000){
		    		m_ui.dir_left->setPixmap(dir_left_pressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
				pan_inc = -1;
		    	}
		    	else if (val == 0){
				pan_inc = 0;
		    		m_ui.dir_left->setPixmap(dir_left_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		    	}
			m_ui.dir_left->move(m_ui.dir_pad->x(), m_ui.dir_pad->y());
			m_ui.dir_right->move(m_ui.dir_pad->x(), m_ui.dir_pad->y());
			//std::printf("D-Pad: %d x, %d y\n", m_ui.dir_pad->x(), m_ui.dir_pad->y());
		}
		if((int) number == 7){		//UP or DOWN - directional pad
		    	if(val >= 30000){
    				m_ui.dir_down->setPixmap(dir_dwn_pressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
				tilt_inc = -1;
		    	}
		    	else if (val == 0){
		    		m_ui.dir_down->setPixmap(dir_dwn_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
				tilt_inc = 0;
		    	}
			if(val <= -30000){
		    		m_ui.dir_up->setPixmap(dir_up_pressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
				tilt_inc = 1;
			}
		    	else if (val == 0){
		    		m_ui.dir_up->setPixmap(dir_up_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
				tilt_inc = 0;
		    	}
			m_ui.dir_up->move(m_ui.dir_pad->x(), m_ui.dir_pad->y());
			m_ui.dir_down->move(m_ui.dir_pad->x(), m_ui.dir_pad->y());
			//std::printf("D-Pad: %d x, %d y\n", m_ui.dir_pad->x(), m_ui.dir_pad->y());
		}
	}
	else if (!usingGamepadControls){
		
	}

	if (type == JS_EVENT_BUTTON && usingGamepadControls){ 	//checking for only button values, not axis.
		if((int) number == 0){
		    	if(val == 1){//if A pressed
		    		m_ui.a_Button->setPixmap(a_p.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
				//m_ui.a_Button->setPixmap(QPixmap(":/test_pressed.png").scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.a_Button->setPixmap(a_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
				
				//m_ui.a_Button->setPixmap(QPixmap(":/test_unpressed.png").scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("A button: %d x, %d y\n", m_ui.a_Button->x(), m_ui.a_Button->y());
		}
		if((int) number == 1){
		    	if(val == 1){//if B pressed
		    		m_ui.b_Button->setPixmap(b_p.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.b_Button->setPixmap(b_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("B button: %d x, %d y\n", m_ui.b_Button->x(), m_ui.b_Button->y());
		}
		if((int) number == 2){
		    	if(val == 1){//if X pressed
		    		m_ui.x_Button->setPixmap(x_p.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.x_Button->setPixmap(x_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("X button: %d x, %d y\n", m_ui.x_Button->x(), m_ui.x_Button->y());
		}
		if((int) number == 3){
		    	if(val == 1){//if Y pressed
		    		m_ui.y_Button->setPixmap(y_p.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.y_Button->setPixmap(y_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("Y button: %d x, %d y\n", m_ui.y_Button->x(), m_ui.y_Button->y());
		}
		if((int) number == 4){
		    	if(val == 1){//if LB pressed
		    		m_ui.LB->setPixmap(LB_p.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.LB->setPixmap(LB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("LB: %d x, %d y\n", m_ui.LB->x(), m_ui.LB->y());
		}
		if((int) number == 5){
		    	if(val == 1){//if RB pressed
		    		m_ui.RB->setPixmap(RB_p.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
				changeCam(90,90);
		    	}
		    	else{
		    		m_ui.RB->setPixmap(RB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("RB: %d x, %d y\n", m_ui.RB->x(), m_ui.RB->y());
		}
		if((int) number == 6){
		    	if(val == 1){//if BACK pressed
		    		m_ui.back_Button->setPixmap(back_p.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.back_Button->setPixmap(back_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("BACK button: %d x, %d y\n", m_ui.back_Button->x(), m_ui.back_Button->y());
		}
		if((int) number == 7){
		    	if(val == 1){//if START pressed
		    		m_ui.start_Button->setPixmap(start_p.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.start_Button->setPixmap(start_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("START button: %d x, %d y\n", m_ui.start_Button->x(), m_ui.start_Button->y());
		}
		if((int) number == 8){
		    	if(val == 1){//if logo button pressed
		    		m_ui.logo_Button->setPixmap(logo_p.scaled(cur_logo_size,cur_logo_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.logo_Button->setPixmap(logo_unp.scaled(cur_logo_size,cur_logo_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("Logo button: %d x, %d y\n", m_ui.logo_Button->x(), m_ui.logo_Button->y());
		}
		if((int) number == 9){
		    	if(val == 1){//if LTB stick pressed down
		    		m_ui.LTB_Button->setPixmap(LTB_p.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.LTB_Button->setPixmap(LTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("Left Thumb Stick: %d x, %d y\n", m_ui.LTB_Button->x(), m_ui.LTB_Button->y());
		}
		if((int) number == 10){
		    	if(val == 1){//if RTB stick pressed down
		    		m_ui.RTB_Button->setPixmap(RTB_p.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		    	}
		    	else{
		    		m_ui.RTB_Button->setPixmap(RTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		    	}
			//std::printf("Right Thumb Stick: %d x, %d y\n", m_ui.RTB_Button->x(), m_ui.RTB_Button->y());
		}
	}
	else if (!usingGamepadControls){
		
	}

	if(usingGamepadControls && ((int) number == 1 || (int) number == 4) && type == JS_EVENT_AXIS){//change throttle
		changeThrottles(valVertLeft, valVertRight);
	}
}

void HelelaniJoystickControls::toggle() //When Gamepad Active/Inactive is pressed...
{
	if(usingGamepadControls){
		usingGamepadControls=false;
		m_ui.toggleGamepad->setText("Gamepad Inactive");
		std::printf("Window Widget: %d x, %d y\n", m_widget->width(), m_widget->height());
		//Grey-out all input buttons, bunkers, and triggers on the GUI
		m_ui.a_Button->setPixmap(a_dis.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.b_Button->setPixmap(b_dis.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.x_Button->setPixmap(x_dis.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.y_Button->setPixmap(y_dis.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.LB->setPixmap(LB_dis.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		m_ui.RB->setPixmap(RB_dis.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		m_ui.LT->setPixmap(LT_dis.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		m_ui.RT->setPixmap(RT_dis.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		m_ui.dir_pad->setPixmap(dir_dis.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_right->setPixmap(dir_right_dis.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_down->setPixmap(dir_dwn_dis.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_up->setPixmap(dir_up_dis.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_left->setPixmap(dir_left_dis.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.start_Button->setPixmap(start_dis.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		m_ui.back_Button->setPixmap(back_dis.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		m_ui.LTB_Button->setPixmap(LTB_dis.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		m_ui.RTB_Button->setPixmap(RTB_dis.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		m_ui.LTB_Placeholder->setPixmap(LTB_dis.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));
		m_ui.RTB_Placeholder->setPixmap(RTB_dis.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));
		m_ui.logo_Button->setPixmap(logo_dis.scaled(cur_logo_size,cur_logo_size,Qt::KeepAspectRatio));
	}
	else{
		usingGamepadControls=true;
		m_ui.toggleGamepad->setText("Gamepad Active");
		//Switch back to normal images
		m_ui.a_Button->setPixmap(a_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.b_Button->setPixmap(b_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.x_Button->setPixmap(x_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.y_Button->setPixmap(y_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.LB->setPixmap(LB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		m_ui.RB->setPixmap(RB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		m_ui.LT->setPixmap(LT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		m_ui.RT->setPixmap(RT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		m_ui.dir_pad->setPixmap(dir_plc.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_right->setPixmap(dir_right_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_down->setPixmap(dir_dwn_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_up->setPixmap(dir_up_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_left->setPixmap(dir_left_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.start_Button->setPixmap(start_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		m_ui.back_Button->setPixmap(back_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		m_ui.LTB_Button->setPixmap(LTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		m_ui.RTB_Button->setPixmap(RTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		m_ui.LTB_Placeholder->setPixmap(LTB_plc.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));
		m_ui.RTB_Placeholder->setPixmap(RTB_plc.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));
		m_ui.logo_Button->setPixmap(logo_unp.scaled(cur_logo_size,cur_logo_size,Qt::KeepAspectRatio));
	}
}

void HelelaniJoystickControls::changeCam(float p, float t){
    helelani_common::CameraCtrl msg = {};
    msg.pan = p;
    msg.tilt = t;
    m_situationCamPub.publish(msg);
}

void HelelaniJoystickControls::changeThrottles(float left, float right)//Publish throttle message
{
    helelani_common::Throttle msg = {};
    msg.left = left;
    msg.right = right;
    m_pub.publish(msg);
}

void HelelaniJoystickControls::checkSitCam(){
    if(pan<=180 && pan>=0 && tilt<=150 && tilt>=30){
	changeCam(pan + pan_inc, tilt + tilt_inc);
    }
    if (pan >= 180){pan = 180;}
    if (pan <= 0){pan = 0;}
    if (tilt >= 150){tilt = 150;}
    if (tilt <= 30){tilt = 30;}
}

void HelelaniJoystickControls::checkWindowSize(){
	//std::printf("test\n");//Test if this function is being called.
	
	//get hyp of the window widget
	new_window_size_x = m_widget->width();
	new_window_size_y = m_widget->height();
	current_window_hyp = sqrt((m_widget->width()*m_widget->width())+(m_widget->height()*m_widget->height()));
	
	//set scaler
	//mult_factor = current_window_hyp / def_window_hyp;
	mult_factor_x = new_window_size_x / default_window_size_x;
	mult_factor_y = new_window_size_y / default_window_size_y;
	mult_factor = mult_factor_x * mult_factor_y;
	
	//set size for visuals
	cur_abxy_size = abxy_button_size * mult_factor;
	cur_bunker_size = bunker_size * mult_factor;
	cur_trigger_size = trigger_size * mult_factor;
	cur_Dpad_size = Dpad_size * mult_factor;
	cur_start_back_size = start_back_button_size * mult_factor;
	cur_logo_size = logo_button_size * mult_factor;
	cur_thumb_bottom_size = thumb_bottom_layer_size * mult_factor;
	cur_thumb_top_size = thumb_top_layer_size * mult_factor;
	cur_toggleGamepadSize_x = toggleGamepadSize_x * mult_factor;
	cur_toggleGamepadSize_y = toggleGamepadSize_y * mult_factor;

	//when resizing window, change visual sizes and (x,y) positions.
	if(new_window_size_x != old_window_size_x || new_window_size_y != old_window_size_y){
		//std::printf("resize detected!\n");
		m_ui.a_Button->resize(cur_abxy_size, cur_abxy_size);
		m_ui.a_Button->setPixmap(a_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.a_Button->move(a_button_x*mult_factor_x, a_button_y*mult_factor_y);

		m_ui.b_Button->resize(cur_abxy_size, cur_abxy_size);
		m_ui.b_Button->setPixmap(b_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.b_Button->move(b_button_x*mult_factor_x, b_button_y*mult_factor_y);

		m_ui.x_Button->resize(cur_abxy_size, cur_abxy_size);
		m_ui.x_Button->setPixmap(x_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.x_Button->move(x_button_x*mult_factor_x, x_button_y*mult_factor_y);

		m_ui.y_Button->resize(cur_abxy_size, cur_abxy_size);
		m_ui.y_Button->setPixmap(y_unp.scaled(cur_abxy_size,cur_abxy_size,Qt::KeepAspectRatio));
		m_ui.y_Button->move(y_button_x*mult_factor_x, y_button_y*mult_factor_y);

		m_ui.LB->resize(cur_bunker_size, cur_bunker_size);
		m_ui.LB->setPixmap(LB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		m_ui.LB->move(LB_x*mult_factor_x, LB_y*mult_factor_y);

		m_ui.RB->resize(cur_bunker_size, cur_bunker_size);
		m_ui.RB->setPixmap(RB_unp.scaled(cur_bunker_size,cur_bunker_size,Qt::KeepAspectRatio));
		m_ui.RB->move(RB_x*mult_factor_x, RB_y*mult_factor_y);

		m_ui.LT->resize(cur_trigger_size, cur_trigger_size);
		m_ui.LT->setPixmap(LT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		m_ui.LT->move(LT_x*mult_factor_x, LT_y*mult_factor_y);

		m_ui.RT->resize(cur_trigger_size, cur_trigger_size);
		m_ui.RT->setPixmap(RT_unp.scaled(cur_trigger_size,cur_trigger_size,Qt::KeepAspectRatio));
		m_ui.RT->move(RT_x*mult_factor_x, RT_y*mult_factor_y);

		m_ui.dir_pad->resize(cur_Dpad_size, cur_Dpad_size);
		m_ui.dir_pad->setPixmap(dir_plc.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_pad->move(dir_pad_x*mult_factor_x, dir_pad_y*mult_factor_y);

		m_ui.dir_down->resize(cur_Dpad_size, cur_Dpad_size);
		m_ui.dir_up->resize(cur_Dpad_size, cur_Dpad_size);
		m_ui.dir_left->resize(cur_Dpad_size, cur_Dpad_size);
		m_ui.dir_right->resize(cur_Dpad_size, cur_Dpad_size);
		m_ui.dir_down->setPixmap(dir_dwn_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
    		m_ui.dir_up->setPixmap(dir_up_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
    		m_ui.dir_left->setPixmap(dir_left_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
    		m_ui.dir_right->setPixmap(dir_right_unpressed.scaled(cur_Dpad_size,cur_Dpad_size,Qt::KeepAspectRatio));
		m_ui.dir_down->move(dir_pad_x*mult_factor_x, dir_pad_y*mult_factor_y);
		m_ui.dir_up->move(dir_pad_x*mult_factor_x, dir_pad_y*mult_factor_y);
		m_ui.dir_left->move(dir_pad_x*mult_factor_x, dir_pad_y*mult_factor_y);
		m_ui.dir_right->move(dir_pad_x*mult_factor_x, dir_pad_y*mult_factor_y);

		m_ui.start_Button->resize(cur_start_back_size, cur_start_back_size);
		m_ui.start_Button->setPixmap(start_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		m_ui.start_Button->move(start_button_x*mult_factor_x, start_button_y*mult_factor_y);

		m_ui.back_Button->resize(cur_start_back_size, cur_start_back_size);
		m_ui.back_Button->setPixmap(back_unp.scaled(cur_start_back_size,cur_start_back_size,Qt::KeepAspectRatio));
		m_ui.back_Button->move(back_button_x*mult_factor_x, back_button_y*mult_factor_y);

		m_ui.logo_Button->resize(cur_logo_size, cur_logo_size);
		m_ui.logo_Button->setPixmap(logo_unp.scaled(cur_logo_size,cur_logo_size,Qt::KeepAspectRatio));
		m_ui.logo_Button->move(logo_Button_x*mult_factor_x, logo_Button_y*mult_factor_y);

		m_ui.LTB_Placeholder->resize(cur_thumb_bottom_size, cur_thumb_bottom_size);
		m_ui.RTB_Placeholder->resize(cur_thumb_bottom_size, cur_thumb_bottom_size);
		m_ui.LTB_Placeholder->setPixmap(LTB_plc.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));
		m_ui.RTB_Placeholder->setPixmap(RTB_plc.scaled(cur_thumb_bottom_size,cur_thumb_bottom_size,Qt::KeepAspectRatio));
		m_ui.LTB_Placeholder->move(LTB_Placeholder_x*mult_factor_x, LTB_Placeholder_y*mult_factor_y);
		m_ui.RTB_Placeholder->move(RTB_Placeholder_x*mult_factor_x, RTB_Placeholder_y*mult_factor_y);

		m_ui.LTB_Button->resize(cur_thumb_top_size, cur_thumb_top_size);
		m_ui.RTB_Button->resize(cur_thumb_top_size, cur_thumb_top_size);
		m_ui.LTB_Button->setPixmap(LTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		m_ui.RTB_Button->setPixmap(RTB_unp.scaled(cur_thumb_top_size,cur_thumb_top_size,Qt::KeepAspectRatio));
		m_ui.LTB_Button->move(LTB_Button_x*mult_factor_x, LTB_Button_y*mult_factor_y);
		m_ui.RTB_Button->move(RTB_Button_x*mult_factor_x, RTB_Button_y*mult_factor_y);

		m_ui.toggleGamepad->resize(cur_toggleGamepadSize_x, cur_toggleGamepadSize_y);
		m_ui.toggleGamepad->move(toggleGamepad_x*mult_factor_x, toggleGamepad_y*mult_factor_y);
		QFont font = m_ui.toggleGamepad->font();
		font.setPointSize(12.5f * mult_factor);
		m_ui.toggleGamepad->setFont(font);
		
	}
	//Remember previous window (x, y) sizes 
	old_window_size_x = new_window_size_x;
	old_window_size_y = new_window_size_y;
}

void Worker::doWork()
{
	unsigned char mType, mNumber;
	short mVal;
	struct js_event e;

   	while(true){
		//std::printf("Waiting for read\n");
		read(fd, &e, sizeof(e));//Wait until a button value has changed
		mType = e.type;
		mNumber = e.number;
		mVal = e.value;
		emit resultReady(mType, mNumber, mVal);
		//std::printf("Result ready:%d, %d\n", (int)mNumber, mVal);
	}
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniJoystickControls, rqt_gui_cpp::Plugin)
