#ifndef DRIVEGAMEPAD_H
#define DRIVEGAMEPAD_H

#include <QGroupBox>

class DriveGamepad : public QGroupBox
{
	Q_OBJECT
public:
	explicit DriveGamepad(QWidget *parent = 0);
	void Update();
	
};
#endif // DRIVEGAMEPAD_H
