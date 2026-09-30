#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtWidgets>
#include "ui_counterDlg.h"
#include "timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();
        virtual ~ejemplo1() = default;

    public slots:
        void doButton();
        void doReset();
        void doSlider(int v);

    private:
        void doCount(int increment);

        Timer mytimer;
        int cont = 0;
};

#endif // ejemplo1_H
