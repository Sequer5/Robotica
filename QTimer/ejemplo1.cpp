#include "ejemplo1.h"
#include <QDebug>

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()));
	// connect(reset_button, SIGNAL(clicked()), this, SLOT(doReset()) );
	connect(slider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)));
	connect(slider, SIGNAL(valueChanged(int)), lcdNumber_2, SLOT(display(int)));

	timer.start(500);
}

void ejemplo1::doButton()
{
	if (timer.isActive()) timer.stop(); else timer.start();
}

void ejemplo1::doCount()
{
	int v = lcdNumber->value();
	lcdNumber->display(++v);
	qDebug() << "Contador: " << v;
}

void ejemplo1::doReset()
{
	lcdNumber->display(0);
}

void ejemplo1::doSlider(int v)
{
	timer.setInterval(v);
}