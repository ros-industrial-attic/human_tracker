/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date October 2012
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/labeler/main_window.hpp"
#include "../include/labeler/roi.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace labeler {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, ROI_LINES(6)
{
    fixed_aspect_ = 0.5;
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    imageDirName = "../images/";
    //saveDirName = "../images/";

    ui.line_source->setText(imageDirName);
    //ui.line_save->setText(saveDirName);

    ui.label_image->setText("Please select 'Image Source Folder' then press 'Load'.");
    
    //Initialize imageNum
    imageNum=0;

    //Initialize rubberband variables
    startPoint.setX(0);
    startPoint.setY(0);
    endPoint.setX(0);
    endPoint.setY(0);

    rubberBandActive =false;
    isLoaded = false;
}

MainWindow::~MainWindow() {}

// This function writes the TRU file using the appropriate variables
void MainWindow::WriteFileTRU(QString filename, Roi roi){

        QFile file(filename);
        if(!file.open(QIODevice::WriteOnly|QIODevice::Text|QIODevice::Append)){
                QMessageBox msgBox;
                msgBox.setText("Unable to open file");
                msgBox.exec();
                return;
        }

        QTextStream out(&file);
        out << "Upper Left X:" <<QString::number(roi.startX())<<"\n";
        out << "Upper Left Y:" <<QString::number(roi.startY())<<"\n";
        out << "Lower Right X:"<<QString::number(roi.endX())<<"\n";
        out << "Lower Right Y:"<<QString::number(roi.endY())<<"\n";
        out << "ROI Label:"<<QString::number(roi.label())<<"\n";
	out << "Randomly Generated:"<<QString(roi.isRandom()?"true":"false")<<"\n";

        file.close();
}

void MainWindow::WriteFileTRU(QString filename){
	//Delete the existing file and writes listRoi to file
	QFile file(filename);
	
	if(file.exists()){
		file.remove();
	}


	//if(file.remove()){
		for(int i = 0; i<listRoi.size();i++){
			WriteFileTRU(filename, listRoi.at(i));
		}
	//}
	//else
	//{
	//	message("error deleting file");
	//}
}

QList<Roi> MainWindow::ReadFileTRU(QString filename){

	QFile file(filename);
	int upperLeftX=0;
	int upperLeftY=0;
	int lowerRightX =0;
	int lowerRightY =0;
	int roiLabel =0;
	bool randomlyGenerated = false;

	listRoi.clear();

        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
                return listRoi;

        int cnt =0;        
	while(!file.atEnd()){
                QString line = file.readLine();
                line = line.section(':',1);
                switch(cnt%ROI_LINES){
                        case 0: upperLeftX = line.toInt();
                                break;
                        case 1: upperLeftY = line.toInt();
                                break;
                        case 2: lowerRightX = line.toInt();
                                break;
                        case 3: lowerRightY = line.toInt();
                                break;
                        case 4: roiLabel = line.toInt();
				break;
			case 5: if(line.contains("true")){
					randomlyGenerated = true;
				}else{
					randomlyGenerated = false;
				}
				
				Roi roi;
				roi.init(upperLeftX,upperLeftY,lowerRightX,lowerRightY,roiLabel,randomlyGenerated);
				listRoi.append(roi);
                                break;
                }
                cnt++;
        }
        file.close();
	//message("ReadFileTRU list size:"+QString::number(listRoi.size()));
	return listRoi;
}

void MainWindow::mousePressEvent(QMouseEvent* event) {
	if (isLoaded)
	{
		//if (!rubberband)
		rubberband = new QRubberBand(QRubberBand::Rectangle, this);
		startPoint = event->pos();
		rubberband->setGeometry(QRect(startPoint,QSize()));
		rubberband->show();
		rubberBandActive = true;
	}
}	

void MainWindow::mouseMoveEvent(QMouseEvent* event) {
	if(isLoaded)
	{

		if(rubberBandActive){
			rubberband->setGeometry(QRect(startPoint,event->pos()).normalized());
		}
	}	
}

void MainWindow::mouseReleaseEvent(QMouseEvent* event) {
	if(isLoaded)
	{

		if(rubberBandActive){
			endPoint = event->pos();
			rubberBandActive = false;
			rubberband->hide();
		}

		//Save the file
		int roiLabel = ui.spinbox_value->value();

		QString filename ="";
		filename = ui.line_source->text();
		if (!filename.endsWith('/')){
			filename.append('/');
		}

		filename += imageList[imageNum];
		filename =filename.replace(".jpg",".tru");
        	//message(filename);
		
		//Create Roi Object
		Roi roi;
		// Loads the point and set upperLeft and lowerRight given ainy opposite corners of the square
		roi.init(startPoint.x(),startPoint.y(),endPoint.x(),endPoint.y(),roiLabel);
		// This is the offset between frame_image and label_image in the GUI
		roi.offset(9,9);
		// This adjust the height by a percent, and sets width to the desired aspect ratio
		roi.grow_force_aspect(fixed_aspect_, 5.0);
		// This checks to ensure that the values are within the image range and corrects for the boundaries
		const QPixmap *temp = ui.label_image->pixmap();
		roi.bounds(temp->width(),temp->height());
                // This verifies that the minimum area of the selected region is reached before adding it the TRU file
                if (roi.area()>0){
                        WriteFileTRU(filename,roi);
                }
		// Read ROI file to get the most up to date ROI values
		//listRoi = ReadFileTRU(filename);

		// This adds the correct number of randomly generated ROI and corrects existing ones
		randomROI(filename);

		//Display the image with the selected ROI
		drawImage();
	}	
}

bool MainWindow::checkOverlap(Roi roi){
	// Returns true when overlap is greater than "overlap" with any existing ROI using smaller of the two ROI
	int overlapPercentage;
	overlapPercentage = ui.horizontalSlider_overlap->value();
	//int randomValue;
	//randomValue = ui.spinBox_random_value->value();
	//int area1 = roi.area();//*overlap;
	int area2 = 0;
	int overlapArea = 0;

	//message("checkOverlap listRoi size:"+QString::number(listRoi.size()));
	for(int i=0; i<listRoi.size(); i++){
		Roi checkRoi = listRoi.at(i);
		if(!checkRoi.isRandom()){
		  area2 = checkRoi.area();
		  if(roi.area()<area2) area2 = roi.area();//*overlap;
			overlapArea = roi.overlapArea(checkRoi);// * 100;
			//message("Area1: " + QString::number(area1)+" Area2: "+QString::number(area2)+" Overlap: "+QString::number(overlapArea));

			if ((overlapArea*100) > (area2*overlapPercentage)){
				//message("Overlap occurred by more than "+QString::number(overlapPercentage));
				return true;
			}		
		}
	}		
	return false;
}

void MainWindow::randomROI(QString filename){
	
	int randomCount;
	int randomValue;
	int existingCount = 0;
	
	//Set values
	randomCount = ui.spinBox_random_count->value();
	randomValue = ui.spinBox_random_value->value();
	
	//Loop through ROI list and verify the number, value and overlap are correct otherwise update ROI listi
	listRoi = ReadFileTRU(filename);

	if(listRoi.size()>0){
	  const QPixmap *temp = ui.label_image->pixmap();
	  int h = temp->height();
	  int w = temp->width();
	  //message("randomROI first for list size "+ listRoi.size());
	  // Checks each random generated ROI and removes it if
	  // 1.  The number of random boxes is too high
	  // 2.  The randomValue does not match the GUI
	  // 3.  The overlap with a non-random generated ROI is > than overlapPercentage from GUI
	  // 4.  The randomly generated roi has unrealistic aspect ratio
	  for(int i=listRoi.size()-1;i>=0; i--){// go backward for deletes to work right
	    Roi roi = listRoi.at(i); 
	    if(roi.endX()>=w || roi.endY()>=h){
		  listRoi.removeAt(i);		  //Remove from list
	    }
	    if(roi.isRandom() && roi.label() == -1){// is random negative example
		if ((existingCount>randomCount) or (randomValue != roi.label()) or checkOverlap(roi)){
		  listRoi.removeAt(i);		  //Remove from list
		}
		else if(roi.startX()<0 || roi.startY()<0){ // this fixes a previous error
		  listRoi.removeAt(i);
		}
		else if(roi.endX()>=ui.label_image->width() || roi.endY()>=ui.label_image->height()){ // this fixes a previous error
		  listRoi.removeAt(i);
		}
		else{
		  existingCount++;
		}
	      }
	    if(roi.isRandom() && roi.label() != -1){ // is random positive example
	      listRoi.removeAt(i);// to make later code simpler, remove all random, positive examples
	    }
	  }
	  
	  WriteFileTRU(filename);
	  listRoi = ReadFileTRU(filename);

	  for(int i=0;i<listRoi.size();i++){
	    Roi roi = listRoi.at(i);
	    if(roi.label() != -1){ // Note: there should be no random positive examples
	      // this is a positive example manually drawn
	      for(int j=0;j<randomCount;j++){
		Roi randomRoi;
		float percent = 5.0;// 5 percent of height
		randomRoi.pinit(roi,percent);
		randomRoi.grow_force_aspect(fixed_aspect_,0.0);
		const QPixmap *temp = ui.label_image->pixmap();
		randomRoi.bounds(temp->width(),temp->height());
		WriteFileTRU(filename,randomRoi);// adds roi at end of file
	      }
	    }
	  }
	  listRoi = ReadFileTRU(filename);

	  
	  //NOTE: count the rois of each type
	  existingCount =0;
	  for(int k=0;k<listRoi.size();k++){
	    Roi roi = listRoi.at(k);
	    if(roi.isRandom() && roi.label() == -1) existingCount++;
	  }
	  

	  // Adds the correct number of random negative ROIs
	  for(int j=0; j<(randomCount-existingCount);j++){
	    //message("randomROI for loop to add randoms");
	    // Add values to list
	    // Generate Random ROI
	    Roi randomRoi;

	    //generate Random ROI that does not overlap
	    randomRoi.init(randomValue, ui.label_image->width(),ui.label_image->height());
	    randomRoi.grow_force_aspect(fixed_aspect_,0.0);
	    const QPixmap *temp = ui.label_image->pixmap();
	    randomRoi.bounds(temp->width(),temp->height());
	    while(checkOverlap(randomRoi) || (randomRoi.area()<=0)){
	      randomRoi.init(randomValue, ui.label_image->width(),ui.label_image->height());
	      randomRoi.grow_force_aspect(fixed_aspect_,0.0);
	      const QPixmap *temp = ui.label_image->pixmap();
	      randomRoi.bounds(temp->width(),temp->height());
	    }
	    //Add to file and list
	    WriteFileTRU(filename,randomRoi);
	    listRoi = ReadFileTRU(filename);
	  }
	}// end if any rois in file
}

void MainWindow::check_display_random(){
	drawImage();
}


void MainWindow::button_source_browse(){

        QString imageDirName = QFileDialog::getExistingDirectory(this,tr("Select Image Folder"),"../images");
	ui.line_source->setText(imageDirName+"/");
}

/*
void MainWindow::button_save_browse(){

        QString saveDirName = QFileDialog::getExistingDirectory(this,tr("Select Save Folder"),"../images");
    	ui.line_save->setText(saveDirName+"/");
}
*/

void MainWindow::button_load(){
	//Load all images into an array
	imageDirName = ui.line_source->text();
	//saveDirName = ui.line_save->text();

	QDirIterator imageDirIt(imageDirName);//, QDirIterator::Subdirectories);

	imageList.clear();

	while (imageDirIt.hasNext()){
		QString imageTemp= imageDirIt.next();
		if(imageTemp.endsWith(".jpg"))
                {	
			int lastSlash = imageTemp.lastIndexOf('/');
			imageList.append(imageTemp.mid(lastSlash+1));
		}
	}
	
	imageList.sort();
	imageNum = 0;
	isLoaded = true;
	
	drawImage();
}

void MainWindow::message(QString msg){
        QMessageBox msgBox;
        msgBox.setText(msg);
        msgBox.exec();
}

void MainWindow::drawImage(){
	QPixmap pixmap;
	QColor colorRed = QColor(255,0,0);
	QColor colorGreen = QColor(0,255,0);
	QColor colorBlue = QColor(0,0,255);

	QPen red(colorRed,2);
	QPen green(colorGreen,2);
	QPen blue(colorBlue,2);

	QPen redDash(colorRed,2,Qt::DotLine);
	QPen greenDash(colorGreen,2,Qt::DotLine);
	QPen blueDash(colorBlue,2,Qt::DotLine);


	bool drawROI = false;
	QString filename = "";

	if (isLoaded){
		filename = imageDirName + imageList.at(imageNum);
		filename = filename.replace(".jpg",".tru");
		QFile truFile(filename);
		drawROI = truFile.exists();
	}

	if (drawROI){

        	listRoi = ReadFileTRU(filename);
        	pixmap.load(imageDirName+imageList.at(imageNum));
        	QPainter painter(&pixmap);		
                //Code to set rectangle color given ROI value and correct pen style if randomly generated	
		for(int i=0;i<listRoi.size(); i++){
			Roi roi = listRoi.at(i); 
			switch(roi.label()){
				case(1):
					if(roi.isRandom()){
						painter.setPen(redDash);
					}
					else{
						painter.setPen(red);
					}
					break;
				case(-1):

					if(roi.isRandom()){
                                                painter.setPen(greenDash);
                                        }
                                        else{
                                                painter.setPen(green);
					}
					break;
				default:if(roi.isRandom()){
						painter.setPen(blueDash);
					}
					else{
						painter.setPen(blue);
					}
			}	
			if (!roi.isRandom() or ui.checkBox_display_random->isChecked()){
				painter.drawRect(roi.startX(),roi.startY(),roi.width(),roi.height());
			}
		}
        	painter.end();
        	ui.label_image -> setPixmap(pixmap);
        	ui.label_image -> show();
	}
	else
	{
		pixmap.load(imageDirName+imageList.at(imageNum));
		ui.label_image -> setPixmap(pixmap);
	}
}
void MainWindow::button_next(){
	//Load Next Image
	if(isLoaded)
	{
		if(imageNum<=(imageList.size()-2)){
			imageNum=imageNum+1;
		}else{
			imageNum=0;
		}	
		drawImage();
	}
}

void MainWindow::button_previous(){
 	//Load Previous Image
	if(isLoaded)
	{
        	if(imageNum>=1){
                	imageNum--;
       		}else{
                	imageNum=imageList.size()-1;
        	}
		drawImage();
	}
}

void MainWindow::button_clear(){

    if(isLoaded)
    {
    	//Clear ROI values from object
   	startPoint.setX(0);
    	startPoint.setY(0);
    	endPoint.setX(0);
    	endPoint.setY(0);


	
	// Load ROI into list file
	QString filename = "";
	filename = imageDirName + imageList.at(imageNum);
	filename = filename.replace(".jpg",".tru");
	
	listRoi = ReadFileTRU(filename);

	if(!listRoi.isEmpty()){
		//Remove off the end if all ROI are displayed
		if(ui.checkBox_display_random->isChecked()){
			listRoi.removeLast();
		}else{//else only remove non-random ROI if they exist 
			bool notFound = true;
			int i = listRoi.size();
			while(i>0 and notFound){
				i--;
				Roi roi = listRoi.at(i);
				if(!roi.isRandom()){
					listRoi.removeAt(i);
					notFound = false;
				}
			}	
		}
	}	

	WriteFileTRU(filename);

	/*

    	//Edit TRU file
    	QString filename="";
    	filename = saveDirName + imageList.at(imageNum);
    	filename = filename.replace(".jpg",".tru");
    	QFile truFile(filename);
    	if(truFile.exists()){
		// Read file into QStringList Object
		if(!truFile.open(QIODevice::ReadOnly|QIODevice::Text))
			return;

		QStringList truList;
		while(!truFile.atEnd()){
			truList.append(truFile.readLine());
		}
		truFile.close();
		// Check size if less than ROI_LINES delete
		if(truList.size()<=ROI_LINES){
        		truFile.remove();
		}
		else{ //Otherwise save without last ROI_LINES

                	if(!truFile.open(QIODevice::WriteOnly|QIODevice::Text|QIODevice::Truncate))
                        	return;

			QTextStream out(&truFile);
			for(int i=0;i<(truList.size()-ROI_LINES);i++){
				//message(truList.at(i));
				out << truList.at(i);
			}
			truFile.close();
		}
    	}
*/
    	drawImage();
    }
}

void MainWindow::button_recalculate(){
	
   //QMessageBox::about(this, tr("recalulate"),tr("Write code to recalculate"));
   
   QString filename ="";
   filename = ui.line_source->text();
   if (!filename.endsWith('/')){
       filename.append('/');
   }

   filename += imageList[imageNum];
   filename =filename.replace(".jpg",".tru");
   //message(filename);


   // This adds the correct number of randomly generated ROI and corrects existing ones
   randomROI(filename);
   randomROI(filename);
   randomROI(filename);

   //Display the image with the selected ROI
   drawImage();


}


void MainWindow::button_recalculate_temp()
{  
  imageNum = 0;
  for(imageNum=0;imageNum<=(imageList.size()-2);imageNum++){
    button_recalculate();
  }
  imageNum=0;
  button_recalculate();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>Image Labeler 0.01</h2><p>Copyright Southwest Research Institute</p><p>This package labels regions of interest in a 640 x 480 image.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/
void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace labeler
