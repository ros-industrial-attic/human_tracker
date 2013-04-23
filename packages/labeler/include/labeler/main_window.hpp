/*
Software License Agreement (BSD License)
 
Copyright (c) 2013, Southwest Research Institute
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the Southwest Research Institute, nor the names
     of its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

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
