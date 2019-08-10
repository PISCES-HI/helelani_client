# helelani_client
ROS nodes for interfacing with PISCES Rover

# Apply changes to code: 

To apply changes to this code, assuming you have it all successfully installed under *rover_workspace/src/helelani_client*, open a command prompt window, from home enter 'cd rover_workspace' and then enter 'catkin_make' to build. *Note: this will not change the code on Github.*

# When adding plugins:

Make sure the new plugins that are being added are included in the CMakeLists.txt, and plugin.xml files.

## CMakeLists.txt:

src/YourPluggin.cpp src/YourPluggin.h (in add_library())

scripts/your_pluggin (in install())

src/YourPluggin.ui (in QT5_WRAP_UI() *if* implementing a GUI)

## plugin.xml: (if adding client code)
*Within:*

  <library path="lib/libhelelani_client"> </library>

*add:*
  
  <class name="Helelani Your Plugin" type="helelani_client::HelelaniYourPlugin" base_class_type="rqt_gui_cpp::Plugin">
        
        <description>Helelani Your Plugin</description>
        
        <qtgui>
            
            <group>
                
                <label>Helelani</label>
                
                <icon type="theme">folder</icon>
                
                <statustip>Plugins related to PISCES Helelani.</statustip>
            
            </group>
            
            <label>Helelani Your Plugin</label>
            
            <statustip>Helelani Your Plugin</statustip>
        
        </qtgui>
    
    </class>

**Where you would save your .h and .cpp files depends if you're working on the client side (PC) or on the rover side (Helelani's NUC in the avionics box). In each case, make sure both files are saved under: *rover_workspace/src/helelani_client* or *rover_workspace/src/helelani_onboard* respectively.**

# To make a looping/update function (example):

## In your .h file:

namespace helelani_client {

...

public slots:

    void yourFunction();
    
...

}

(Compare with other .h source files for more info)

## In your .cpp file:

namespace helelani_client {

void HelelaniYourPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {

...

connect(timer, SIGNAL(timeout()), this, SLOT(yourFunction()));

...

}

void HelelaniYourPlugin::shutdownPlugin()

{
    
    m_pub.shutdown();
    
    m_situationCamPub.shutdown();

}

...

void HelelaniYourPlugin::yourFunction(){

    //...and do what you want here! It will constantly be called. :)
    
}

(Compare with other .cpp source files for more info)

**This kind of loop can be used to check for events such as when a GUI window is resized! See HelelaniJoystickControls source files for an example - just be aware that there's a lot of code in the .cpp file.**

