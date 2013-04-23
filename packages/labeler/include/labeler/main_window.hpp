/**
 * @file /include/labeler/main_window.hpp
 *
 * @brief Qt based gui for labeler.
 *
 * @date October 2012
 **/
#ifndef labeler_MAIN_WINDOW_H
#define labeler_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QtGui/QMainWindow>
#include <QDirIterator>
#include <QStringList>
#include "ui_main_window.h"
#include <QRubberBand>
#include <QMouseEvent>
#include <QPoint>
#include "roi.hpp"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace labeler {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    void closeEvent(QCloseEvent *event); // Overloaded function
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void drawImage();
    void randomROI();
    void message(QString msg);

public slots:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();

    /******************************************
    ** Manual connections
    *******************************************/
    void button_next();
    void button_previous();
    void button_clear();
    void button_load();
    void button_source_browse();
    void button_recalculate();
    void button_recalculate_temp();
    //void button_save_browse();
    void check_display_random();

private:
    float fixed_aspect_;
    void WriteFileTRU(QString filename, Roi roi);
    void WriteFileTRU(QString filename);
    QList<Roi> ReadFileTRU(QString filename);
    void randomROI(QString filename);
    bool checkOverlap(Roi roi);
    Ui::MainWindowDesign ui;
    QStringList imageList;
    QString imageDirName;
    //QString saveDirName;
    int imageNum;
    QRubberBand *rubberband;
    QPoint startPoint;
    QPoint endPoint;
    bool rubberBandActive;
    bool isLoaded;
    const int ROI_LINES;
    QList<Roi> listRoi;
};

}  // namespace labeler

#endif // labeler_MAIN_WINDOW_H
