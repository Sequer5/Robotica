#include "ejemplo1.h"
#include <QDebug>

ejemplo1::ejemplo1(): Ui_Counter()
{
    setupUi(this);
    show();

    connect(button,       SIGNAL(clicked()),         this, SLOT(doButton()));
    connect(reset_button, SIGNAL(clicked()),          this, SLOT(doReset()));
    connect(slider,       SIGNAL(valueChanged(int)),  this, SLOT(doSlider(int)));
    connect(slider,       SIGNAL(valueChanged(int)),  lcdNumber_2, SLOT(display(int)));


    mytimer.connect(std::bind(&ejemplo1::doCount, this, 1));

    mytimer.start(slider->value());
    lcdNumber_2->display(slider->value());
}

void ejemplo1::doButton()
{
    if (mytimer.isActive())
    {
        mytimer.stop();
        button->setText("START");
    }
    else
    {
        mytimer.start(slider->value());
        button->setText("STOP");
    }
}

void ejemplo1::doCount(int increment)
{
    cont += increment;
    lcdNumber->display(cont);
}

void ejemplo1::doReset()
{
    cont = 0;
    lcdNumber->display(cont);
}

void ejemplo1::doSlider(int v)
{
    mytimer.setInterval(v);
}
